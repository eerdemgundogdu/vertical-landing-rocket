#!/usr/bin/env python3
"""
Telemetry Node for Vertical Landing Rocket
===========================================
Aggregates telemetry data and broadcasts to Ground Control Station via UDP.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt8, Float32MultiArray
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

import socket
import struct
import json
import time
from dataclasses import dataclass, asdict
from typing import Optional


@dataclass
class RocketTelemetry:
    """Complete rocket telemetry packet."""
    timestamp: float
    flight_state: int
    armed: bool
    
    # Position (NED)
    pos_n: float
    pos_e: float
    pos_d: float
    altitude: float  # -pos_d
    
    # Velocity (NED)
    vel_n: float
    vel_e: float
    vel_d: float
    
    # Attitude (quaternion)
    qw: float
    qx: float
    qy: float
    qz: float
    
    # Euler angles (derived)
    roll: float
    pitch: float
    yaw: float
    
    # Angular rates
    gyro_x: float
    gyro_y: float
    gyro_z: float
    
    # System
    battery_voltage: float
    cpu_temp: float
    sensor_status: int
    
    # Guidance
    guidance_phase: int
    target_roll: float
    target_pitch: float


class TelemetryNode(Node):
    """ROS2 node for telemetry aggregation and GCS broadcast."""
    
    def __init__(self):
        super().__init__('telemetry_node')
        
        # Parameters
        self.declare_parameter('gcs_ip', '192.168.1.100')
        self.declare_parameter('gcs_port', 14550)
        self.declare_parameter('broadcast_rate', 20.0)  # Hz
        self.declare_parameter('json_mode', True)  # True for JSON, False for binary
        
        gcs_ip = self.get_parameter('gcs_ip').value
        gcs_port = self.get_parameter('gcs_port').value
        broadcast_rate = self.get_parameter('broadcast_rate').value
        self.json_mode = self.get_parameter('json_mode').value
        
        # UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gcs_addr = (gcs_ip, gcs_port)
        
        # Telemetry state
        self.telemetry = RocketTelemetry(
            timestamp=0.0, flight_state=0, armed=False,
            pos_n=0.0, pos_e=0.0, pos_d=0.0, altitude=0.0,
            vel_n=0.0, vel_e=0.0, vel_d=0.0,
            qw=1.0, qx=0.0, qy=0.0, qz=0.0,
            roll=0.0, pitch=0.0, yaw=0.0,
            gyro_x=0.0, gyro_y=0.0, gyro_z=0.0,
            battery_voltage=0.0, cpu_temp=0.0, sensor_status=0,
            guidance_phase=0, target_roll=0.0, target_pitch=0.0
        )
        
        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/rocket/odometry', self.odometry_callback, qos)
        self.state_sub = self.create_subscription(
            UInt8, '/rocket/flight_state', self.state_callback, qos)
        self.armed_sub = self.create_subscription(
            Bool, '/rocket/armed', self.armed_callback, qos)
        self.battery_sub = self.create_subscription(
            Vector3, '/rocket/battery', self.battery_callback, qos)
        self.guidance_sub = self.create_subscription(
            Float32MultiArray, '/rocket/guidance_status', self.guidance_callback, qos)
        
        # Broadcast timer
        period = 1.0 / broadcast_rate
        self.timer = self.create_timer(period, self.broadcast_telemetry)
        
        self.get_logger().info(f'Telemetry node broadcasting to {gcs_ip}:{gcs_port}')
    
    def odometry_callback(self, msg: Odometry):
        """Update telemetry from odometry."""
        self.telemetry.pos_n = msg.pose.pose.position.x
        self.telemetry.pos_e = msg.pose.pose.position.y
        self.telemetry.pos_d = msg.pose.pose.position.z
        self.telemetry.altitude = -msg.pose.pose.position.z
        
        self.telemetry.vel_n = msg.twist.twist.linear.x
        self.telemetry.vel_e = msg.twist.twist.linear.y
        self.telemetry.vel_d = msg.twist.twist.linear.z
        
        self.telemetry.qw = msg.pose.pose.orientation.w
        self.telemetry.qx = msg.pose.pose.orientation.x
        self.telemetry.qy = msg.pose.pose.orientation.y
        self.telemetry.qz = msg.pose.pose.orientation.z
        
        self.telemetry.gyro_x = msg.twist.twist.angular.x
        self.telemetry.gyro_y = msg.twist.twist.angular.y
        self.telemetry.gyro_z = msg.twist.twist.angular.z
        
        # Calculate Euler angles
        self.telemetry.roll, self.telemetry.pitch, self.telemetry.yaw = \
            self.quaternion_to_euler(
                self.telemetry.qw, self.telemetry.qx, 
                self.telemetry.qy, self.telemetry.qz)
    
    def state_callback(self, msg: UInt8):
        """Update flight state."""
        self.telemetry.flight_state = msg.data
    
    def armed_callback(self, msg: Bool):
        """Update armed status."""
        self.telemetry.armed = msg.data
    
    def battery_callback(self, msg: Vector3):
        """Update battery info."""
        self.telemetry.battery_voltage = msg.x
        self.telemetry.cpu_temp = msg.y
        self.telemetry.sensor_status = int(msg.z)
    
    def guidance_callback(self, msg: Float32MultiArray):
        """Update guidance status."""
        if len(msg.data) >= 7:
            self.telemetry.guidance_phase = int(msg.data[0])
            self.telemetry.target_roll = msg.data[5]
            self.telemetry.target_pitch = msg.data[6]
    
    def broadcast_telemetry(self):
        """Broadcast telemetry to GCS."""
        self.telemetry.timestamp = time.time()
        
        try:
            if self.json_mode:
                # JSON format (human readable, good for debugging)
                data = json.dumps(asdict(self.telemetry)).encode('utf-8')
            else:
                # Binary format (compact, better for bandwidth)
                data = self.pack_telemetry()
            
            self.sock.sendto(data, self.gcs_addr)
        except Exception as e:
            self.get_logger().error(f'Broadcast error: {e}')
    
    def pack_telemetry(self) -> bytes:
        """Pack telemetry into binary format."""
        # Pack as: timestamp, state, armed, pos(3), vel(3), quat(4), gyro(3), battery, temp
        return struct.pack(
            '<dBB3f3f4f3fff',
            self.telemetry.timestamp,
            self.telemetry.flight_state,
            1 if self.telemetry.armed else 0,
            self.telemetry.pos_n, self.telemetry.pos_e, self.telemetry.pos_d,
            self.telemetry.vel_n, self.telemetry.vel_e, self.telemetry.vel_d,
            self.telemetry.qw, self.telemetry.qx, self.telemetry.qy, self.telemetry.qz,
            self.telemetry.gyro_x, self.telemetry.gyro_y, self.telemetry.gyro_z,
            self.telemetry.battery_voltage, self.telemetry.cpu_temp
        )
    
    @staticmethod
    def quaternion_to_euler(w: float, x: float, y: float, z: float):
        """Convert quaternion to Euler angles."""
        import math
        
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def destroy_node(self):
        """Clean up resources."""
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
