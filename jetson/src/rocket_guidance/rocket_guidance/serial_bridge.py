#!/usr/bin/env python3
"""
Serial Bridge Node for Vertical Landing Rocket
===============================================
Communicates with Teensy 4.1 flight controller via COBS-encoded serial protocol.
Translates between ROS2 topics and binary serial messages.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, UInt8
from geometry_msgs.msg import Vector3, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry

import serial
import struct
import threading
from typing import Optional
from dataclasses import dataclass
from cobs import cobs


# Message IDs (must match firmware)
class MsgId:
    TELEMETRY_STATE = 0x01
    TELEMETRY_RAW = 0x02
    TELEMETRY_STATUS = 0x03
    TELEMETRY_GPS = 0x04
    CMD_ARM = 0x10
    CMD_DISARM = 0x11
    CMD_ABORT = 0x12
    CMD_SET_TARGET = 0x14
    RESP_ACK = 0x20
    RESP_NACK = 0x21


SYNC_BYTE = 0xAA


@dataclass
class TelemetryState:
    timestamp_ms: int
    pos_n: float
    pos_e: float
    pos_d: float
    vel_n: float
    vel_e: float
    vel_d: float
    qw: float
    qx: float
    qy: float
    qz: float
    gyro_x: float
    gyro_y: float
    gyro_z: float


@dataclass
class TelemetryStatus:
    timestamp_ms: int
    flight_state: int
    arm_status: int
    sensor_status: int
    error_code: int
    battery_voltage: float
    cpu_temp: float


class SerialBridge(Node):
    """ROS2 node bridging serial communication with Teensy flight controller."""
    
    def __init__(self):
        super().__init__('serial_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('timeout', 0.1)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        timeout = self.get_parameter('timeout').value
        
        # Serial port
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        
        # Threading
        self.rx_thread: Optional[threading.Thread] = None
        self.running = False
        self.rx_buffer = bytearray()
        
        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Publishers (telemetry from Teensy)
        self.odom_pub = self.create_publisher(Odometry, '/rocket/odometry', qos)
        self.status_pub = self.create_publisher(UInt8, '/rocket/flight_state', qos)
        self.armed_pub = self.create_publisher(Bool, '/rocket/armed', qos)
        self.battery_pub = self.create_publisher(Vector3, '/rocket/battery', qos)
        
        # Subscribers (commands to Teensy)
        self.arm_sub = self.create_subscription(
            Bool, '/rocket/cmd/arm', self.arm_callback, qos)
        self.abort_sub = self.create_subscription(
            Bool, '/rocket/cmd/abort', self.abort_callback, qos)
        self.target_sub = self.create_subscription(
            Vector3, '/rocket/attitude_cmd', self.target_callback, qos)
        
        # Connect to serial
        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            self.connected = True
            self.get_logger().info(f'Connected to {port} at {baudrate} baud')
            
            # Start receive thread
            self.running = True
            self.rx_thread = threading.Thread(target=self.receive_loop, daemon=True)
            self.rx_thread.start()
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
        
        # Heartbeat timer
        self.create_timer(1.0, self.heartbeat_callback)
    
    def receive_loop(self):
        """Background thread for receiving serial data."""
        while self.running and self.serial:
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self.process_received_data(data)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
    
    def process_received_data(self, data: bytes):
        """Process incoming serial data, extracting COBS-encoded packets."""
        self.rx_buffer.extend(data)
        
        # Look for packet delimiters (0x00)
        while b'\x00' in self.rx_buffer:
            idx = self.rx_buffer.index(b'\x00'[0])
            packet = bytes(self.rx_buffer[:idx])
            self.rx_buffer = self.rx_buffer[idx + 1:]
            
            if len(packet) > 0:
                try:
                    decoded = cobs.decode(packet)
                    self.process_packet(decoded)
                except Exception as e:
                    pass  # Invalid COBS, ignore
    
    def process_packet(self, data: bytes):
        """Process a decoded packet."""
        if len(data) < 4:
            return
        
        if data[0] != SYNC_BYTE:
            return
        
        # Verify CRC
        rx_crc = struct.unpack('<H', data[-2:])[0]
        calc_crc = self.crc16(data[:-2])
        if rx_crc != calc_crc:
            return
        
        msg_id = data[1]
        payload = data[2:-2]
        
        if msg_id == MsgId.TELEMETRY_STATE:
            self.handle_telemetry_state(payload)
        elif msg_id == MsgId.TELEMETRY_STATUS:
            self.handle_telemetry_status(payload)
    
    def handle_telemetry_state(self, payload: bytes):
        """Handle state telemetry message."""
        if len(payload) < 52:
            return
        
        # Unpack state data
        values = struct.unpack('<I13f', payload[:56])
        
        state = TelemetryState(
            timestamp_ms=values[0],
            pos_n=values[1], pos_e=values[2], pos_d=values[3],
            vel_n=values[4], vel_e=values[5], vel_d=values[6],
            qw=values[7], qx=values[8], qy=values[9], qz=values[10],
            gyro_x=values[11], gyro_y=values[12], gyro_z=values[13]
        )
        
        # Publish as Odometry
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.child_frame_id = 'rocket'
        
        msg.pose.pose.position.x = state.pos_n
        msg.pose.pose.position.y = state.pos_e
        msg.pose.pose.position.z = state.pos_d
        
        msg.pose.pose.orientation.w = state.qw
        msg.pose.pose.orientation.x = state.qx
        msg.pose.pose.orientation.y = state.qy
        msg.pose.pose.orientation.z = state.qz
        
        msg.twist.twist.linear.x = state.vel_n
        msg.twist.twist.linear.y = state.vel_e
        msg.twist.twist.linear.z = state.vel_d
        
        msg.twist.twist.angular.x = state.gyro_x
        msg.twist.twist.angular.y = state.gyro_y
        msg.twist.twist.angular.z = state.gyro_z
        
        self.odom_pub.publish(msg)
    
    def handle_telemetry_status(self, payload: bytes):
        """Handle status telemetry message."""
        if len(payload) < 16:
            return
        
        values = struct.unpack('<IBBBB2f', payload[:16])
        
        status = TelemetryStatus(
            timestamp_ms=values[0],
            flight_state=values[1],
            arm_status=values[2],
            sensor_status=values[3],
            error_code=values[4],
            battery_voltage=values[5],
            cpu_temp=values[6]
        )
        
        # Publish flight state
        state_msg = UInt8()
        state_msg.data = status.flight_state
        self.status_pub.publish(state_msg)
        
        # Publish armed status
        armed_msg = Bool()
        armed_msg.data = status.arm_status != 0
        self.armed_pub.publish(armed_msg)
        
        # Publish battery info
        battery_msg = Vector3()
        battery_msg.x = status.battery_voltage
        battery_msg.y = status.cpu_temp
        battery_msg.z = float(status.sensor_status)
        self.battery_pub.publish(battery_msg)
    
    def arm_callback(self, msg: Bool):
        """Handle arm/disarm command."""
        if msg.data:
            # Send ARM command with key
            payload = struct.pack('<I', 0xDEADBEEF)
            self.send_packet(MsgId.CMD_ARM, payload)
            self.get_logger().info('Sending ARM command')
        else:
            self.send_packet(MsgId.CMD_DISARM, b'')
            self.get_logger().info('Sending DISARM command')
    
    def abort_callback(self, msg: Bool):
        """Handle abort command."""
        if msg.data:
            payload = struct.pack('<B', 1)  # Manual abort
            self.send_packet(MsgId.CMD_ABORT, payload)
            self.get_logger().warn('Sending ABORT command!')
    
    def target_callback(self, msg: Vector3):
        """Handle attitude target command."""
        payload = struct.pack('<4f', msg.x, msg.y, msg.z, 0.0)
        self.send_packet(MsgId.CMD_SET_TARGET, payload)
    
    def send_packet(self, msg_id: int, payload: bytes):
        """Send a packet to the flight controller."""
        if not self.connected or not self.serial:
            return
        
        # Build packet
        packet = bytearray()
        packet.append(SYNC_BYTE)
        packet.append(msg_id)
        packet.extend(payload)
        
        # Add CRC
        crc = self.crc16(bytes(packet))
        packet.extend(struct.pack('<H', crc))
        
        # COBS encode
        encoded = cobs.encode(bytes(packet))
        
        # Send with delimiter
        try:
            self.serial.write(encoded + b'\x00')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')
    
    def heartbeat_callback(self):
        """Periodic heartbeat for connection monitoring."""
        if not self.connected:
            self.get_logger().warn('Serial not connected')
    
    @staticmethod
    def crc16(data: bytes) -> int:
        """Calculate CRC16-CCITT."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc
    
    def destroy_node(self):
        """Clean up resources."""
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
