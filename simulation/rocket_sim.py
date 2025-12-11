#!/usr/bin/env python3
"""
Rocket Simulator - Software-In-The-Loop (SITL)
===============================================
6-DOF flight dynamics simulation for vertical landing rocket.
Includes atmospheric model, thrust curve, and ground interaction.
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from dataclasses import dataclass, field
from typing import Tuple, Callable, Optional
import argparse
import socket
import json
import time


@dataclass
class RocketParams:
    """Physical parameters of the rocket."""
    mass_wet: float = 1.5       # Total wet mass (kg)
    mass_dry: float = 1.2       # Dry mass without propellant (kg)
    length: float = 0.6         # Total length (m)
    diameter: float = 0.054     # Body diameter (m)
    
    # Aerodynamics
    cd: float = 0.5             # Drag coefficient
    
    # Motor
    thrust_avg: float = 40.0    # Average thrust (N)
    burn_time: float = 1.5      # Burn duration (s)
    
    # TVC
    max_gimbal: float = 15.0    # Max gimbal angle (deg)
    
    # Moments of inertia (kg·m²) - approximate as cylinder
    @property
    def Ixx(self) -> float:
        return 0.5 * self.mass_wet * (self.diameter / 2) ** 2
    
    @property
    def Iyy(self) -> float:
        return (1/12) * self.mass_wet * (3 * (self.diameter / 2) ** 2 + self.length ** 2)
    
    @property
    def Izz(self) -> float:
        return self.Iyy
    
    @property
    def area(self) -> float:
        return np.pi * (self.diameter / 2) ** 2


@dataclass
class Environment:
    """Atmospheric and environmental parameters."""
    g: float = 9.81                     # Gravity (m/s²)
    rho_sl: float = 1.225               # Sea level air density (kg/m³)
    scale_height: float = 8500.0        # Scale height (m)
    wind_n: float = 0.0                 # Wind velocity North (m/s)
    wind_e: float = 0.0                 # Wind velocity East (m/s)
    
    def air_density(self, altitude: float) -> float:
        """Calculate air density at altitude."""
        return self.rho_sl * np.exp(-altitude / self.scale_height)


@dataclass
class RocketState:
    """Full state vector of the rocket."""
    # Position (NED frame, m)
    pos_n: float = 0.0
    pos_e: float = 0.0
    pos_d: float = 0.0  # Negative = above ground
    
    # Velocity (NED frame, m/s)
    vel_n: float = 0.0
    vel_e: float = 0.0
    vel_d: float = 0.0
    
    # Attitude (quaternion)
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    
    # Angular velocity (body frame, rad/s)
    p: float = 0.0
    q: float = 0.0
    r: float = 0.0
    
    def to_array(self) -> np.ndarray:
        """Convert to numpy array for integration."""
        return np.array([
            self.pos_n, self.pos_e, self.pos_d,
            self.vel_n, self.vel_e, self.vel_d,
            self.qw, self.qx, self.qy, self.qz,
            self.p, self.q, self.r
        ])
    
    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'RocketState':
        """Create from numpy array."""
        return cls(
            pos_n=arr[0], pos_e=arr[1], pos_d=arr[2],
            vel_n=arr[3], vel_e=arr[4], vel_d=arr[5],
            qw=arr[6], qx=arr[7], qy=arr[8], qz=arr[9],
            p=arr[10], q=arr[11], r=arr[12]
        )
    
    @property
    def altitude(self) -> float:
        return -self.pos_d
    
    def get_euler(self) -> Tuple[float, float, float]:
        """Get Euler angles (roll, pitch, yaw) in radians."""
        # Roll
        sinr_cosp = 2 * (self.qw * self.qx + self.qy * self.qz)
        cosr_cosp = 1 - 2 * (self.qx**2 + self.qy**2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # Pitch
        sinp = 2 * (self.qw * self.qy - self.qz * self.qx)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)
        
        # Yaw
        siny_cosp = 2 * (self.qw * self.qz + self.qx * self.qy)
        cosy_cosp = 1 - 2 * (self.qy**2 + self.qz**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


class RocketSimulator:
    """6-DOF rocket flight simulator."""
    
    def __init__(self, params: RocketParams = None, env: Environment = None):
        self.params = params or RocketParams()
        self.env = env or Environment()
        
        self.state = RocketState()
        self.time = 0.0
        
        # Control inputs
        self.gimbal_x = 0.0  # Gimbal angle X (rad)
        self.gimbal_y = 0.0  # Gimbal angle Y (rad)
        
        # Flight phase
        self.launched = False
        self.landed = False
        self.motor_on = False
        self.burnout_time = None
        
        # History for plotting
        self.history = {
            't': [], 'alt': [], 'vel_d': [],
            'roll': [], 'pitch': [], 'yaw': [],
            'pos_n': [], 'pos_e': []
        }
    
    def thrust(self, t: float) -> float:
        """Get thrust at time t."""
        if not self.launched:
            return 0.0
        
        flight_time = t
        if flight_time < 0:
            return 0.0
        elif flight_time < self.params.burn_time:
            self.motor_on = True
            return self.params.thrust_avg
        else:
            if self.motor_on:
                self.burnout_time = t
            self.motor_on = False
            return 0.0
    
    def mass(self, t: float) -> float:
        """Get mass at time t."""
        if not self.launched:
            return self.params.mass_wet
        
        flight_time = t
        if flight_time <= 0:
            return self.params.mass_wet
        elif flight_time >= self.params.burn_time:
            return self.params.mass_dry
        else:
            # Linear mass decrease during burn
            dm = (self.params.mass_wet - self.params.mass_dry) / self.params.burn_time
            return self.params.mass_wet - dm * flight_time
    
    def derivatives(self, t: float, y: np.ndarray) -> np.ndarray:
        """Compute state derivatives for integration."""
        state = RocketState.from_array(y)
        
        # Get current mass
        m = self.mass(t)
        
        # Get thrust
        T = self.thrust(t)
        
        # Quaternion to rotation matrix (body to NED)
        qw, qx, qy, qz = state.qw, state.qx, state.qy, state.qz
        
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])
        
        # Thrust force in body frame (pointing up = -Z in body)
        # Apply gimbal angles
        thrust_body = np.array([
            T * np.sin(self.gimbal_y),   # X (roll)
            T * np.sin(self.gimbal_x),   # Y (pitch)
            -T * np.cos(self.gimbal_x) * np.cos(self.gimbal_y)  # Z (up)
        ])
        
        # Transform to NED
        thrust_ned = R @ thrust_body
        
        # Aerodynamic drag
        vel_air = np.array([
            state.vel_n - self.env.wind_n,
            state.vel_e - self.env.wind_e,
            state.vel_d
        ])
        speed = np.linalg.norm(vel_air)
        
        rho = self.env.air_density(state.altitude)
        q_dyn = 0.5 * rho * speed**2
        drag_mag = q_dyn * self.params.cd * self.params.area
        
        if speed > 0.1:
            drag_ned = -drag_mag * vel_air / speed
        else:
            drag_ned = np.zeros(3)
        
        # Gravity
        gravity_ned = np.array([0, 0, m * self.env.g])
        
        # Ground interaction
        if state.altitude <= 0 and state.vel_d > 0:
            # On ground or below, moving down
            ground_force = np.array([0, 0, -m * self.env.g - 1000 * state.vel_d])
            
            # Friction for horizontal motion
            friction_coef = 0.5
            horiz_vel = np.sqrt(state.vel_n**2 + state.vel_e**2)
            if horiz_vel > 0.1:
                friction = friction_coef * m * self.env.g
                ground_force[0] = -friction * state.vel_n / horiz_vel
                ground_force[1] = -friction * state.vel_e / horiz_vel
        else:
            ground_force = np.zeros(3)
        
        # Total force
        F_total = thrust_ned + drag_ned + gravity_ned + ground_force
        
        # Acceleration
        acc = F_total / m
        
        # Angular dynamics (simplified)
        # Thrust moment arm for gimbal
        moment_arm = self.params.length * 0.3  # Approximate
        
        M_thrust = np.array([
            moment_arm * T * np.sin(self.gimbal_y),
            moment_arm * T * np.sin(self.gimbal_x),
            0.0
        ])
        
        # Damping
        damping = 0.1
        M_damp = -damping * np.array([state.p, state.q, state.r])
        
        M_total = M_thrust + M_damp
        
        # Angular acceleration
        alpha = np.array([
            M_total[0] / self.params.Ixx,
            M_total[1] / self.params.Iyy,
            M_total[2] / self.params.Izz
        ])
        
        # Quaternion derivative
        omega = np.array([state.p, state.q, state.r])
        q_dot = 0.5 * np.array([
            -qx*omega[0] - qy*omega[1] - qz*omega[2],
            qw*omega[0] + qy*omega[2] - qz*omega[1],
            qw*omega[1] + qz*omega[0] - qx*omega[2],
            qw*omega[2] + qx*omega[1] - qy*omega[0]
        ])
        
        # State derivatives
        dydt = np.array([
            state.vel_n, state.vel_e, state.vel_d,  # Position derivatives
            acc[0], acc[1], acc[2],                  # Velocity derivatives
            q_dot[0], q_dot[1], q_dot[2], q_dot[3],  # Quaternion derivatives
            alpha[0], alpha[1], alpha[2]             # Angular velocity derivatives
        ])
        
        return dydt
    
    def step(self, dt: float):
        """Advance simulation by dt seconds."""
        if self.landed:
            return
        
        # Integrate
        y0 = self.state.to_array()
        sol = solve_ivp(
            self.derivatives,
            (self.time, self.time + dt),
            y0,
            method='RK45',
            max_step=dt/10
        )
        
        # Update state
        self.state = RocketState.from_array(sol.y[:, -1])
        self.time += dt
        
        # Normalize quaternion
        q_norm = np.sqrt(self.state.qw**2 + self.state.qx**2 + 
                         self.state.qy**2 + self.state.qz**2)
        if q_norm > 0:
            self.state.qw /= q_norm
            self.state.qx /= q_norm
            self.state.qy /= q_norm
            self.state.qz /= q_norm
        
        # Ground constraint
        if self.state.altitude < 0:
            self.state.pos_d = 0
            if abs(self.state.vel_d) < 0.5 and not self.motor_on:
                self.landed = True
        
        # Record history
        roll, pitch, yaw = self.state.get_euler()
        self.history['t'].append(self.time)
        self.history['alt'].append(self.state.altitude)
        self.history['vel_d'].append(-self.state.vel_d)
        self.history['roll'].append(np.degrees(roll))
        self.history['pitch'].append(np.degrees(pitch))
        self.history['yaw'].append(np.degrees(yaw))
        self.history['pos_n'].append(self.state.pos_n)
        self.history['pos_e'].append(self.state.pos_e)
    
    def launch(self):
        """Trigger launch."""
        self.launched = True
        print(f"[{self.time:.2f}s] LAUNCH!")
    
    def set_gimbal(self, x: float, y: float):
        """Set gimbal angles (degrees)."""
        max_rad = np.radians(self.params.max_gimbal)
        self.gimbal_x = np.clip(np.radians(x), -max_rad, max_rad)
        self.gimbal_y = np.clip(np.radians(y), -max_rad, max_rad)
    
    def simple_controller(self):
        """Simple PD controller for stabilization."""
        roll, pitch, yaw = self.state.get_euler()
        
        # Target: upright (roll=0, pitch=0)
        kp = 5.0
        kd = 0.5
        
        pitch_cmd = -kp * pitch - kd * self.state.q
        roll_cmd = -kp * roll - kd * self.state.p
        
        self.set_gimbal(np.degrees(pitch_cmd), np.degrees(roll_cmd))
    
    def run(self, duration: float = 30.0, dt: float = 0.01, 
            visualize: bool = False, broadcast: bool = False):
        """Run simulation for specified duration."""
        print("Starting simulation...")
        print(f"  Duration: {duration}s")
        print(f"  Time step: {dt}s")
        
        # Launch after 1 second
        launch_time = 1.0
        
        # UDP broadcast setup
        sock = None
        if broadcast:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            gcs_addr = ('127.0.0.1', 14550)
            print(f"  Broadcasting to {gcs_addr}")
        
        steps = int(duration / dt)
        
        for i in range(steps):
            # Launch trigger
            if self.time >= launch_time and not self.launched:
                self.launch()
            
            # Run controller
            if self.launched and not self.landed:
                self.simple_controller()
            
            # Step simulation
            self.step(dt)
            
            # Broadcast telemetry
            if broadcast and sock and i % 10 == 0:
                roll, pitch, yaw = self.state.get_euler()
                telem = {
                    'timestamp': time.time(),
                    'flight_state': 2 if self.motor_on else (6 if self.landed else 3),
                    'armed': True,
                    'altitude': self.state.altitude,
                    'vel_d': self.state.vel_d,
                    'roll': roll,
                    'pitch': pitch,
                    'yaw': yaw,
                    'battery_voltage': 7.8,
                    'cpu_temp': 45.0
                }
                try:
                    sock.sendto(json.dumps(telem).encode(), gcs_addr)
                except:
                    pass
            
            # Print status periodically
            if i % int(1.0 / dt) == 0:
                roll, pitch, yaw = self.state.get_euler()
                print(f"[{self.time:5.1f}s] Alt: {self.state.altitude:6.1f}m  "
                      f"Vel: {-self.state.vel_d:+6.1f}m/s  "
                      f"Pitch: {np.degrees(pitch):+5.1f}°  "
                      f"T: {self.thrust(self.time):5.1f}N")
            
            if self.landed:
                print(f"\n[{self.time:.2f}s] LANDED!")
                print(f"  Final position: N={self.state.pos_n:.2f}m, E={self.state.pos_e:.2f}m")
                break
        
        if sock:
            sock.close()
        
        if visualize:
            self.plot()
        
        return self.history
    
    def plot(self):
        """Plot simulation results."""
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('Vertical Landing Rocket Simulation', fontsize=14)
        
        # Altitude
        axes[0, 0].plot(self.history['t'], self.history['alt'], 'b-', linewidth=2)
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Altitude (m)')
        axes[0, 0].set_title('Altitude vs Time')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].axhline(y=0, color='brown', linestyle='--', alpha=0.5)
        
        # Velocity
        axes[0, 1].plot(self.history['t'], self.history['vel_d'], 'r-', linewidth=2)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Vertical Velocity (m/s)')
        axes[0, 1].set_title('Vertical Velocity vs Time')
        axes[0, 1].grid(True, alpha=0.3)
        axes[0, 1].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        
        # Attitude
        axes[1, 0].plot(self.history['t'], self.history['roll'], label='Roll')
        axes[1, 0].plot(self.history['t'], self.history['pitch'], label='Pitch')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Angle (deg)')
        axes[1, 0].set_title('Attitude vs Time')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        # Ground track
        axes[1, 1].plot(self.history['pos_e'], self.history['pos_n'], 'g-', linewidth=2)
        axes[1, 1].plot(0, 0, 'ro', markersize=10, label='Launch')
        axes[1, 1].plot(self.history['pos_e'][-1], self.history['pos_n'][-1], 
                        'b^', markersize=10, label='Landing')
        axes[1, 1].set_xlabel('East (m)')
        axes[1, 1].set_ylabel('North (m)')
        axes[1, 1].set_title('Ground Track')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        axes[1, 1].axis('equal')
        
        plt.tight_layout()
        plt.savefig('simulation_results.png', dpi=150)
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Rocket SITL Simulator')
    parser.add_argument('--visualize', '-v', action='store_true',
                        help='Show plots after simulation')
    parser.add_argument('--broadcast', '-b', action='store_true',
                        help='Broadcast telemetry to GCS')
    parser.add_argument('--duration', '-d', type=float, default=30.0,
                        help='Simulation duration (seconds)')
    parser.add_argument('--wind', '-w', type=float, default=0.0,
                        help='Wind speed (m/s)')
    
    args = parser.parse_args()
    
    # Create environment with optional wind
    env = Environment(wind_n=args.wind)
    
    # Create and run simulator
    sim = RocketSimulator(env=env)
    sim.run(duration=args.duration, visualize=args.visualize, broadcast=args.broadcast)


if __name__ == '__main__':
    main()
