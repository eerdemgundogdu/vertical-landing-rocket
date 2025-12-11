#!/usr/bin/env python3
"""
Object Detection Node for Vertical Landing Rocket
==================================================
YOLO-based obstacle detection for safe landing zone identification.
Detects obstacles, hazards, and evaluates landing site safety.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

import cv2
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum


class ObstacleClass(Enum):
    """Obstacle classification types."""
    UNKNOWN = 0
    PERSON = 1
    VEHICLE = 2
    BUILDING = 3
    TREE = 4
    ROCK = 5
    WATER = 6
    LANDING_PAD = 7
    SAFE_ZONE = 8


@dataclass
class Detection:
    """Single detection result."""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x, y, w, h
    center: Tuple[int, int]
    is_obstacle: bool


@dataclass
class LandingZoneAnalysis:
    """Analysis of potential landing zone."""
    is_safe: bool
    safety_score: float  # 0-1
    obstacles_detected: int
    closest_obstacle_dist: float  # meters
    recommended_offset: Tuple[float, float]  # N, E offset in meters


class ObjectDetectionNode(Node):
    """ROS2 node for YOLO-based object detection."""
    
    # Classes that are obstacles (hazards for landing)
    OBSTACLE_CLASSES = {'person', 'car', 'truck', 'bus', 'motorcycle', 'bicycle',
                        'dog', 'cat', 'bird', 'horse', 'sheep', 'cow', 'elephant',
                        'bear', 'zebra', 'giraffe'}
    
    def __init__(self):
        super().__init__('object_detection_node')
        
        # Parameters
        self.declare_parameter('model_path', 'yolov8n.onnx')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_threshold', 0.4)
        self.declare_parameter('input_size', 640)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('landing_zone_radius', 5.0)  # meters
        
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.nms_threshold = self.get_parameter('nms_threshold').value
        self.input_size = self.get_parameter('input_size').value
        camera_topic = self.get_parameter('camera_topic').value
        self.publish_debug = self.get_parameter('publish_debug').value
        self.landing_radius = self.get_parameter('landing_zone_radius').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Load YOLO model
        self.net = None
        self.class_names = []
        self.load_model(model_path)
        
        # State
        self.detections: List[Detection] = []
        self.landing_analysis: Optional[LandingZoneAnalysis] = None
        
        # QoS
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, qos)
        
        # Publishers
        self.detections_pub = self.create_publisher(
            Float32MultiArray, '/rocket/detections', qos)
        self.landing_safe_pub = self.create_publisher(
            Bool, '/rocket/landing_safe', qos)
        self.obstacle_pub = self.create_publisher(
            Point, '/rocket/closest_obstacle', qos)
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image, '/rocket/detection/debug_image', qos)
        
        self.get_logger().info('Object detection node initialized')
    
    def load_model(self, model_path: str):
        """Load YOLO model (ONNX format)."""
        try:
            self.net = cv2.dnn.readNetFromONNX(model_path)
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            self.get_logger().info(f'Loaded YOLO model from {model_path}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load YOLO model: {e}')
            self.get_logger().warn('Using fallback simple detection')
            self.net = None
        
        # COCO class names
        self.class_names = [
            'person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train',
            'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign',
            'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep',
            'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella',
            'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard',
            'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard',
            'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork',
            'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair',
            'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv',
            'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
            'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
            'scissors', 'teddy bear', 'hair drier', 'toothbrush'
        ]
    
    def image_callback(self, msg: Image):
        """Process incoming camera images for object detection."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        # Run detection
        self.detections = self.detect_objects(cv_image)
        
        # Analyze landing zone
        self.landing_analysis = self.analyze_landing_zone(cv_image, self.detections)
        
        # Publish results
        self.publish_detections()
        self.publish_landing_safety()
        
        # Debug visualization
        if self.publish_debug:
            debug_image = self.draw_detections(cv_image, self.detections)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
    
    def detect_objects(self, image: np.ndarray) -> List[Detection]:
        """Run YOLO object detection on image."""
        detections = []
        
        if self.net is None:
            # Fallback: simple color-based detection (for testing without model)
            return self.fallback_detection(image)
        
        h, w = image.shape[:2]
        
        # Preprocess
        blob = cv2.dnn.blobFromImage(
            image, 1/255.0, (self.input_size, self.input_size),
            swapRB=True, crop=False
        )
        
        # Run inference
        self.net.setInput(blob)
        outputs = self.net.forward()
        
        # Parse outputs (YOLOv8 format)
        outputs = outputs[0].T
        
        boxes = []
        confidences = []
        class_ids = []
        
        x_scale = w / self.input_size
        y_scale = h / self.input_size
        
        for row in outputs:
            classes_scores = row[4:]
            max_score = np.max(classes_scores)
            
            if max_score >= self.conf_threshold:
                class_id = np.argmax(classes_scores)
                
                # Get bounding box
                cx, cy, bw, bh = row[:4]
                x = int((cx - bw/2) * x_scale)
                y = int((cy - bh/2) * y_scale)
                width = int(bw * x_scale)
                height = int(bh * y_scale)
                
                boxes.append([x, y, width, height])
                confidences.append(float(max_score))
                class_ids.append(int(class_id))
        
        # Non-maximum suppression
        if boxes:
            indices = cv2.dnn.NMSBoxes(boxes, confidences, 
                                        self.conf_threshold, self.nms_threshold)
            
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                class_id = class_ids[i]
                class_name = self.class_names[class_id] if class_id < len(self.class_names) else 'unknown'
                
                detections.append(Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=confidences[i],
                    bbox=(x, y, w, h),
                    center=(x + w // 2, y + h // 2),
                    is_obstacle=class_name in self.OBSTACLE_CLASSES
                ))
        
        return detections
    
    def fallback_detection(self, image: np.ndarray) -> List[Detection]:
        """Simple fallback detection without neural network."""
        detections = []
        
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Detect red objects (potential hazards)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                detections.append(Detection(
                    class_id=-1,
                    class_name='unknown_obstacle',
                    confidence=0.5,
                    bbox=(x, y, w, h),
                    center=(x + w // 2, y + h // 2),
                    is_obstacle=True
                ))
        
        return detections
    
    def analyze_landing_zone(self, image: np.ndarray, 
                             detections: List[Detection]) -> LandingZoneAnalysis:
        """Analyze image for landing zone safety."""
        h, w = image.shape[:2]
        center_x, center_y = w // 2, h // 2
        
        obstacles = [d for d in detections if d.is_obstacle]
        num_obstacles = len(obstacles)
        
        if num_obstacles == 0:
            return LandingZoneAnalysis(
                is_safe=True,
                safety_score=1.0,
                obstacles_detected=0,
                closest_obstacle_dist=float('inf'),
                recommended_offset=(0.0, 0.0)
            )
        
        # Find closest obstacle to center
        min_dist = float('inf')
        closest_center = (center_x, center_y)
        
        for obs in obstacles:
            dx = obs.center[0] - center_x
            dy = obs.center[1] - center_y
            dist = np.sqrt(dx**2 + dy**2)
            if dist < min_dist:
                min_dist = dist
                closest_center = obs.center
        
        # Convert pixel distance to approximate meters (assuming known altitude)
        # This is a rough estimate - would need proper camera calibration
        pixel_per_meter = w / (2 * self.landing_radius)  # Approximate
        closest_dist_m = min_dist / pixel_per_meter
        
        # Calculate safety score
        if closest_dist_m > self.landing_radius:
            safety_score = 1.0
        else:
            safety_score = closest_dist_m / self.landing_radius
        
        # Calculate recommended offset to avoid obstacles
        if closest_dist_m < self.landing_radius:
            # Move away from closest obstacle
            dx = center_x - closest_center[0]
            dy = center_y - closest_center[1]
            norm = np.sqrt(dx**2 + dy**2)
            if norm > 0:
                offset_n = (dy / norm) * (self.landing_radius - closest_dist_m)
                offset_e = (dx / norm) * (self.landing_radius - closest_dist_m)
            else:
                offset_n, offset_e = 2.0, 0.0  # Default offset north
        else:
            offset_n, offset_e = 0.0, 0.0
        
        return LandingZoneAnalysis(
            is_safe=safety_score > 0.7,
            safety_score=safety_score,
            obstacles_detected=num_obstacles,
            closest_obstacle_dist=closest_dist_m,
            recommended_offset=(offset_n, offset_e)
        )
    
    def publish_detections(self):
        """Publish detection results."""
        msg = Float32MultiArray()
        
        # Format: [num_detections, class_id1, conf1, x1, y1, w1, h1, ...]
        data = [float(len(self.detections))]
        for det in self.detections:
            data.extend([
                float(det.class_id),
                det.confidence,
                float(det.bbox[0]),
                float(det.bbox[1]),
                float(det.bbox[2]),
                float(det.bbox[3])
            ])
        
        msg.data = data
        self.detections_pub.publish(msg)
    
    def publish_landing_safety(self):
        """Publish landing zone safety status."""
        if self.landing_analysis:
            # Safety status
            safe_msg = Bool()
            safe_msg.data = self.landing_analysis.is_safe
            self.landing_safe_pub.publish(safe_msg)
            
            # Closest obstacle
            if self.landing_analysis.obstacles_detected > 0:
                point_msg = Point()
                point_msg.x = self.landing_analysis.recommended_offset[1]  # East
                point_msg.y = self.landing_analysis.recommended_offset[0]  # North
                point_msg.z = self.landing_analysis.closest_obstacle_dist
                self.obstacle_pub.publish(point_msg)
    
    def draw_detections(self, image: np.ndarray, 
                        detections: List[Detection]) -> np.ndarray:
        """Draw detection boxes on image."""
        output = image.copy()
        
        for det in detections:
            x, y, w, h = det.bbox
            
            # Color based on obstacle status
            if det.is_obstacle:
                color = (0, 0, 255)  # Red for obstacles
            else:
                color = (0, 255, 0)  # Green for safe objects
            
            # Draw bounding box
            cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)
            
            # Draw label
            label = f'{det.class_name}: {det.confidence:.2f}'
            cv2.putText(output, label, (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw landing zone info
        if self.landing_analysis:
            h, w = output.shape[:2]
            status_color = (0, 255, 0) if self.landing_analysis.is_safe else (0, 0, 255)
            status_text = f"SAFE: {self.landing_analysis.safety_score:.2f}"
            cv2.putText(output, status_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 2)
            
            # Draw landing zone circle
            cv2.circle(output, (w // 2, h // 2), 
                       int(self.landing_radius * w / 10), (0, 255, 255), 2)
        
        return output


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
