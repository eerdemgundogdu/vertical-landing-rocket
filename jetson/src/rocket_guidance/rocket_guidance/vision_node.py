#!/usr/bin/env python3
"""
Vision Node for Vertical Landing Rocket
========================================
Computer vision for landing target detection using ArUco markers.
Provides relative pose estimation to the landing pad.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool
from cv_bridge import CvBridge

import cv2
import numpy as np
from typing import Optional, Tuple


class VisionNode(Node):
    """ROS2 node for landing pad detection using ArUco markers."""
    
    def __init__(self):
        super().__init__('vision_node')
        
        # Parameters
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.2)  # Marker size in meters
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('publish_debug', True)
        
        aruco_dict_name = self.get_parameter('aruco_dict').value
        self.marker_size = self.get_parameter('marker_size').value
        camera_topic = self.get_parameter('camera_topic').value
        self.publish_debug = self.get_parameter('publish_debug').value
        
        # ArUco detector setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, aruco_dict_name))
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # State
        self.target_detected = False
        self.last_detection_time = 0.0
        
        # QoS
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, camera_topic, self.image_callback, qos)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, qos)
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseStamped, '/rocket/landing_target_pose', qos)
        self.detection_pub = self.create_publisher(
            Bool, '/rocket/target_detected', qos)
        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image, '/rocket/vision/debug_image', qos)
        
        self.get_logger().info(f'Vision node initialized with marker size {self.marker_size}m')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsic parameters."""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration received')
    
    def image_callback(self, msg: Image):
        """Process incoming camera images for marker detection."""
        if self.camera_matrix is None:
            return  # Wait for camera calibration
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        
        # Detect ArUco markers
        corners, ids, rejected = self.detector.detectMarkers(cv_image)
        
        if ids is not None and len(ids) > 0:
            # Use the first detected marker as landing target
            marker_corners = corners[0]
            marker_id = ids[0][0]
            
            # Estimate pose
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [marker_corners], self.marker_size, 
                self.camera_matrix, self.dist_coeffs)
            
            if rvecs is not None and tvecs is not None:
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]
                
                # Publish pose
                self.publish_pose(tvec, rvec, msg.header.stamp)
                
                self.target_detected = True
                self.last_detection_time = self.get_clock().now().nanoseconds / 1e9
                
                # Debug visualization
                if self.publish_debug:
                    debug_image = cv_image.copy()
                    cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
                    cv2.drawFrameAxes(debug_image, self.camera_matrix, 
                                      self.dist_coeffs, rvec, tvec, 0.1)
                    
                    # Add text overlay
                    dist = np.linalg.norm(tvec)
                    cv2.putText(debug_image, f'ID: {marker_id} Dist: {dist:.2f}m',
                                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
                    debug_msg.header = msg.header
                    self.debug_image_pub.publish(debug_msg)
        else:
            self.target_detected = False
        
        # Publish detection status
        detection_msg = Bool()
        detection_msg.data = self.target_detected
        self.detection_pub.publish(detection_msg)
    
    def publish_pose(self, tvec: np.ndarray, rvec: np.ndarray, stamp):
        """Publish the pose of the landing target relative to camera."""
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'camera_frame'
        
        # Translation (camera frame: X-right, Y-down, Z-forward)
        msg.pose.position.x = float(tvec[0])
        msg.pose.position.y = float(tvec[1])
        msg.pose.position.z = float(tvec[2])
        
        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = self.rotation_matrix_to_quaternion(rotation_matrix)
        
        msg.pose.orientation.w = quat[0]
        msg.pose.orientation.x = quat[1]
        msg.pose.orientation.y = quat[2]
        msg.pose.orientation.z = quat[3]
        
        self.pose_pub.publish(msg)
    
    @staticmethod
    def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
        """Convert 3x3 rotation matrix to quaternion (w, x, y, z)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            s = 2.0 * np.sqrt(trace + 1.0)
            w = 0.25 * s
            x = (R[2, 1] - R[1, 2]) / s
            y = (R[0, 2] - R[2, 0]) / s
            z = (R[1, 0] - R[0, 1]) / s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return (w, x, y, z)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
