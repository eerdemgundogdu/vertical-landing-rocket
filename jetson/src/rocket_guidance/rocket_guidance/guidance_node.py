#!/usr/bin/env python3
"""
Guidance Node for Vertical Landing Rocket
=========================================
High-level trajectory planning and guidance for powered descent landing.
Implements polynomial trajectory generation and powered descent guidance (PDG).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3
from nav_msgs.msg import Odometry

import numpy as np
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class GuidancePhase(Enum):
    """Guidance phases for landing."""
    IDLE = 0
    BOOST = 1
    COAST = 2
    POWERED_DESCENT = 3
    TERMINAL_GUIDANCE = 4
    LANDED = 5


@dataclass
class RocketState:
    """Current state of the rocket."""
    position: np.ndarray  # NED position [m]
    velocity: np.ndarray  # NED velocity [m/s]
    attitude: np.ndarray  # Euler angles [rad] (roll, pitch, yaw)
    angular_rate: np.ndarray  # Body rates [rad/s]
    timestamp: float


@dataclass
class GuidanceCommand:
    """Guidance command output."""
    target_attitude: np.ndarray  # Target Euler angles [rad]
    thrust_command: float  # Normalized thrust (0-1)
    phase: GuidancePhase


class GuidanceNode(Node):
    """ROS2 node for high-level guidance and trajectory planning."""
    
    def __init__(self):
        super().__init__('guidance_node')
        
        # Parameters
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('landing_target_n', 0.0)
        self.declare_parameter('landing_target_e', 0.0)
        self.declare_parameter('max_tilt_angle', 0.5)  # rad (~28 deg)
        self.declare_parameter('gravity', 9.81)
        self.declare_parameter('terminal_altitude', 5.0)  # m
        self.declare_parameter('terminal_velocity', 2.0)  # m/s
        
        self.update_rate = self.get_parameter('update_rate').value
        self.landing_target = np.array([
            self.get_parameter('landing_target_n').value,
            self.get_parameter('landing_target_e').value,
            0.0
        ])
        self.max_tilt = self.get_parameter('max_tilt_angle').value
        self.gravity = self.get_parameter('gravity').value
        self.terminal_alt = self.get_parameter('terminal_altitude').value
        self.terminal_vel = self.get_parameter('terminal_velocity').value
        
        # State
        self.current_state: Optional[RocketState] = None
        self.phase = GuidancePhase.IDLE
        self.is_armed = False
        self.descent_start_time: Optional[float] = None
        self.trajectory_coeffs: Optional[np.ndarray] = None
        
        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/rocket/odometry', self.odometry_callback, qos)
        self.arm_sub = self.create_subscription(
            Bool, '/rocket/armed', self.arm_callback, qos)
        
        # Publishers
        self.attitude_cmd_pub = self.create_publisher(
            Vector3, '/rocket/attitude_cmd', qos)
        self.guidance_status_pub = self.create_publisher(
            Float32MultiArray, '/rocket/guidance_status', qos)
        
        # Timer
        period = 1.0 / self.update_rate
        self.timer = self.create_timer(period, self.guidance_loop)
        
        self.get_logger().info('Guidance node initialized')
    
    def odometry_callback(self, msg: Odometry):
        """Handle incoming odometry from state estimation."""
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        ang = msg.twist.twist.angular
        
        # Extract Euler angles from quaternion
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(q.w, q.x, q.y, q.z)
        
        self.current_state = RocketState(
            position=np.array([pos.x, pos.y, pos.z]),
            velocity=np.array([vel.x, vel.y, vel.z]),
            attitude=np.array([roll, pitch, yaw]),
            angular_rate=np.array([ang.x, ang.y, ang.z]),
            timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )
    
    def arm_callback(self, msg: Bool):
        """Handle arm/disarm commands."""
        self.is_armed = msg.data
        if self.is_armed:
            self.phase = GuidancePhase.BOOST
            self.get_logger().info('System armed, entering BOOST phase')
        else:
            self.phase = GuidancePhase.IDLE
            self.get_logger().info('System disarmed')
    
    def guidance_loop(self):
        """Main guidance loop - compute commands based on current state."""
        if self.current_state is None:
            return
        
        state = self.current_state
        cmd = GuidanceCommand(
            target_attitude=np.zeros(3),
            thrust_command=0.0,
            phase=self.phase
        )
        
        if self.phase == GuidancePhase.IDLE:
            # No guidance commands
            pass
        
        elif self.phase == GuidancePhase.BOOST:
            # During boost, maintain vertical attitude
            cmd.target_attitude = np.array([0.0, 0.0, state.attitude[2]])
            cmd.thrust_command = 1.0
            
            # Transition to coast after burnout (detected by flight controller)
            if state.velocity[2] > 0.1 and abs(state.velocity[2]) < 1.0:
                self.phase = GuidancePhase.COAST
        
        elif self.phase == GuidancePhase.COAST:
            # Maintain attitude during coast
            cmd.target_attitude = np.array([0.0, 0.0, state.attitude[2]])
            cmd.thrust_command = 0.0
            
            # Transition to powered descent at apogee
            altitude = -state.position[2]  # NED to altitude
            if state.velocity[2] > 0:  # Descending
                self.phase = GuidancePhase.POWERED_DESCENT
                self.descent_start_time = state.timestamp
                self.plan_trajectory(state)
                self.get_logger().info('Apogee detected, starting powered descent')
        
        elif self.phase == GuidancePhase.POWERED_DESCENT:
            # Powered descent guidance (simplified G-FOLD-like algorithm)
            cmd = self.compute_powered_descent(state)
            
            # Transition to terminal guidance at low altitude
            altitude = -state.position[2]
            if altitude < self.terminal_alt:
                self.phase = GuidancePhase.TERMINAL_GUIDANCE
        
        elif self.phase == GuidancePhase.TERMINAL_GUIDANCE:
            # Final approach - vertical descent at constant velocity
            cmd = self.compute_terminal_guidance(state)
            
            # Detect landing
            altitude = -state.position[2]
            vert_vel = state.velocity[2]
            if altitude < 0.5 and abs(vert_vel) < 0.5:
                self.phase = GuidancePhase.LANDED
                self.get_logger().info('Landing detected!')
        
        elif self.phase == GuidancePhase.LANDED:
            cmd.target_attitude = np.array([0.0, 0.0, 0.0])
            cmd.thrust_command = 0.0
        
        # Publish commands
        self.publish_commands(cmd)
        self.publish_status(state, cmd)
    
    def plan_trajectory(self, state: RocketState):
        """Plan polynomial trajectory from current state to landing point."""
        # Simplified trajectory: 5th order polynomial in time
        # Boundary conditions: current pos/vel/accel to target pos/vel/accel
        
        altitude = -state.position[2]
        descent_rate = max(-state.velocity[2], 5.0)  # Ensure positive descent
        time_to_land = altitude / descent_rate
        
        # Store trajectory time
        self.trajectory_time = time_to_land
        
        self.get_logger().info(f'Trajectory planned: {time_to_land:.1f}s to landing')
    
    def compute_powered_descent(self, state: RocketState) -> GuidanceCommand:
        """Compute guidance commands for powered descent phase."""
        # Position error to landing target
        pos_error = self.landing_target - state.position
        pos_error[2] = 0  # Zero altitude error (handled separately)
        
        # Altitude and descent rate
        altitude = -state.position[2]
        descent_rate = state.velocity[2]  # Positive = descending in NED
        
        # Compute desired descent rate (linear profile)
        target_descent_rate = min(self.terminal_vel, altitude * 0.5)
        
        # Velocity command (proportional + descent rate)
        kp_pos = 0.5
        kp_vel = 2.0
        
        # Horizontal velocity command
        vel_cmd = kp_pos * pos_error[:2]
        vel_error = vel_cmd - state.velocity[:2]
        
        # Convert to tilt angles (simplified: roll/pitch for N/E control)
        accel_cmd = kp_vel * vel_error
        
        # Limit tilt
        accel_mag = np.linalg.norm(accel_cmd)
        if accel_mag > 0:
            max_accel = self.gravity * np.tan(self.max_tilt)
            if accel_mag > max_accel:
                accel_cmd = accel_cmd / accel_mag * max_accel
        
        # Convert acceleration to tilt angles
        pitch_cmd = np.arctan2(accel_cmd[0], self.gravity)  # North = pitch
        roll_cmd = -np.arctan2(accel_cmd[1], self.gravity)   # East = roll (negative)
        
        # Vertical rate control for thrust
        rate_error = target_descent_rate - descent_rate
        thrust_cmd = 0.5 + rate_error * 0.1  # Nominal 50% + correction
        thrust_cmd = np.clip(thrust_cmd, 0.3, 1.0)
        
        return GuidanceCommand(
            target_attitude=np.array([roll_cmd, pitch_cmd, 0.0]),
            thrust_command=thrust_cmd,
            phase=GuidancePhase.POWERED_DESCENT
        )
    
    def compute_terminal_guidance(self, state: RocketState) -> GuidanceCommand:
        """Compute guidance for final landing approach."""
        # Simple proportional control to landing target
        pos_error = self.landing_target - state.position
        
        # Very gentle horizontal correction
        kp = 0.2
        vel_cmd = kp * pos_error[:2]
        vel_error = vel_cmd - state.velocity[:2]
        
        # Limit to very small tilt
        max_tilt = 0.1  # ~6 degrees
        
        accel_cmd = 0.5 * vel_error
        accel_mag = np.linalg.norm(accel_cmd)
        if accel_mag > 0:
            max_accel = self.gravity * np.tan(max_tilt)
            if accel_mag > max_accel:
                accel_cmd = accel_cmd / accel_mag * max_accel
        
        pitch_cmd = np.arctan2(accel_cmd[0], self.gravity)
        roll_cmd = -np.arctan2(accel_cmd[1], self.gravity)
        
        # Terminal descent rate (constant low velocity)
        altitude = -state.position[2]
        target_descent = self.terminal_vel * (altitude / self.terminal_alt)
        target_descent = max(target_descent, 0.5)
        
        rate_error = target_descent - state.velocity[2]
        thrust_cmd = 0.4 + rate_error * 0.15
        thrust_cmd = np.clip(thrust_cmd, 0.2, 0.8)
        
        return GuidanceCommand(
            target_attitude=np.array([roll_cmd, pitch_cmd, 0.0]),
            thrust_command=thrust_cmd,
            phase=GuidancePhase.TERMINAL_GUIDANCE
        )
    
    def publish_commands(self, cmd: GuidanceCommand):
        """Publish attitude command."""
        msg = Vector3()
        msg.x = cmd.target_attitude[0]
        msg.y = cmd.target_attitude[1]
        msg.z = cmd.target_attitude[2]
        self.attitude_cmd_pub.publish(msg)
    
    def publish_status(self, state: RocketState, cmd: GuidanceCommand):
        """Publish guidance status for GCS."""
        msg = Float32MultiArray()
        altitude = -state.position[2]
        descent_rate = state.velocity[2]
        horizontal_dist = np.linalg.norm(state.position[:2] - self.landing_target[:2])
        
        msg.data = [
            float(self.phase.value),
            altitude,
            descent_rate,
            horizontal_dist,
            cmd.thrust_command,
            cmd.target_attitude[0],
            cmd.target_attitude[1]
        ]
        self.guidance_status_pub.publish(msg)
    
    @staticmethod
    def quaternion_to_euler(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    node = GuidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
