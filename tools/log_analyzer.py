#!/usr/bin/env python3
"""
Flight Log Analyzer
===================
Tool for analyzing binary flight log files from the rocket.
Generates plots and statistics from recorded data.
"""

import struct
import numpy as np
import matplotlib.pyplot as plt
import argparse
from pathlib import Path
from dataclasses import dataclass
from typing import List, BinaryIO


# Log entry types (must match firmware)
LOG_TYPE_IMU = 0x01
LOG_TYPE_BARO = 0x02
LOG_TYPE_GPS = 0x03
LOG_TYPE_STATE = 0x04
LOG_TYPE_EVENT = 0x05


@dataclass
class ImuEntry:
    timestamp: float  # seconds
    accel_x: float
    accel_y: float
    accel_z: float
    gyro_x: float
    gyro_y: float
    gyro_z: float


@dataclass
class BaroEntry:
    timestamp: float
    pressure: float
    altitude: float
    temperature: float


@dataclass  
class StateEntry:
    timestamp: float
    pos_n: float
    pos_e: float
    pos_d: float
    vel_n: float
    vel_e: float
    vel_d: float
    roll: float
    pitch: float
    yaw: float
    flight_state: int


class FlightLogAnalyzer:
    """Analyzer for binary flight log files."""
    
    def __init__(self, filepath: str):
        self.filepath = Path(filepath)
        self.imu_data: List[ImuEntry] = []
        self.baro_data: List[BaroEntry] = []
        self.state_data: List[StateEntry] = []
        self.events: List[dict] = []
        
    def parse(self):
        """Parse binary log file."""
        print(f"Parsing {self.filepath}...")
        
        with open(self.filepath, 'rb') as f:
            while True:
                # Read header
                header = f.read(5)
                if len(header) < 5:
                    break
                
                log_type = header[0]
                timestamp = struct.unpack('<I', header[1:5])[0] / 1e6  # Convert to seconds
                
                if log_type == LOG_TYPE_IMU:
                    data = f.read(24)  # 6 floats
                    if len(data) < 24:
                        break
                    values = struct.unpack('<6f', data)
                    self.imu_data.append(ImuEntry(
                        timestamp=timestamp,
                        accel_x=values[0], accel_y=values[1], accel_z=values[2],
                        gyro_x=values[3], gyro_y=values[4], gyro_z=values[5]
                    ))
                
                elif log_type == LOG_TYPE_BARO:
                    data = f.read(12)  # 3 floats
                    if len(data) < 12:
                        break
                    values = struct.unpack('<3f', data)
                    self.baro_data.append(BaroEntry(
                        timestamp=timestamp,
                        pressure=values[0], altitude=values[1], temperature=values[2]
                    ))
                
                elif log_type == LOG_TYPE_STATE:
                    data = f.read(37)  # 9 floats + 1 byte
                    if len(data) < 37:
                        break
                    values = struct.unpack('<9fB', data)
                    self.state_data.append(StateEntry(
                        timestamp=timestamp,
                        pos_n=values[0], pos_e=values[1], pos_d=values[2],
                        vel_n=values[3], vel_e=values[4], vel_d=values[5],
                        roll=values[6], pitch=values[7], yaw=values[8],
                        flight_state=values[9]
                    ))
                
                elif log_type == LOG_TYPE_EVENT:
                    data = f.read(9)  # event_type + 8 bytes
                    if len(data) < 9:
                        break
                    event_type = data[0]
                    self.events.append({
                        'timestamp': timestamp,
                        'type': event_type,
                        'data': data[1:]
                    })
                
                else:
                    # Unknown type, try to skip
                    break
        
        print(f"  IMU entries: {len(self.imu_data)}")
        print(f"  Baro entries: {len(self.baro_data)}")
        print(f"  State entries: {len(self.state_data)}")
        print(f"  Events: {len(self.events)}")
    
    def analyze(self):
        """Generate analysis statistics."""
        print("\n=== FLIGHT ANALYSIS ===\n")
        
        if self.state_data:
            # Find key events
            times = [s.timestamp for s in self.state_data]
            altitudes = [-s.pos_d for s in self.state_data]
            vel_d = [s.vel_d for s in self.state_data]
            
            max_alt = max(altitudes)
            max_alt_time = times[altitudes.index(max_alt)]
            
            # Flight duration
            flight_start = times[0]
            flight_end = times[-1]
            duration = flight_end - flight_start
            
            print(f"Flight Duration: {duration:.2f} s")
            print(f"Max Altitude: {max_alt:.2f} m at T+{max_alt_time:.2f}s")
            print(f"Max Descent Rate: {max(vel_d):.2f} m/s")
            
            # Landing position
            final_state = self.state_data[-1]
            print(f"Landing Position: N={final_state.pos_n:.2f}m, E={final_state.pos_e:.2f}m")
            print(f"Landing Velocity: {final_state.vel_d:.2f} m/s")
        
        if self.imu_data:
            accels = [np.sqrt(i.accel_x**2 + i.accel_y**2 + i.accel_z**2) 
                      for i in self.imu_data]
            print(f"Max Acceleration: {max(accels):.2f} m/s²")
    
    def plot(self, save_path: str = None):
        """Generate plots of flight data."""
        fig, axes = plt.subplots(3, 2, figsize=(14, 10))
        fig.suptitle(f'Flight Log Analysis: {self.filepath.name}', fontsize=14)
        
        # Altitude
        if self.state_data:
            times = [s.timestamp for s in self.state_data]
            altitudes = [-s.pos_d for s in self.state_data]
            axes[0, 0].plot(times, altitudes, 'b-', linewidth=1)
            axes[0, 0].set_xlabel('Time (s)')
            axes[0, 0].set_ylabel('Altitude (m)')
            axes[0, 0].set_title('Altitude')
            axes[0, 0].grid(True, alpha=0.3)
        
        # Velocity
        if self.state_data:
            vel_n = [s.vel_n for s in self.state_data]
            vel_e = [s.vel_e for s in self.state_data]
            vel_d = [-s.vel_d for s in self.state_data]  # Convert to climb rate
            axes[0, 1].plot(times, vel_d, 'r-', label='Vertical', linewidth=1)
            axes[0, 1].plot(times, vel_n, 'g-', label='North', linewidth=0.5, alpha=0.7)
            axes[0, 1].plot(times, vel_e, 'b-', label='East', linewidth=0.5, alpha=0.7)
            axes[0, 1].set_xlabel('Time (s)')
            axes[0, 1].set_ylabel('Velocity (m/s)')
            axes[0, 1].set_title('Velocity')
            axes[0, 1].legend()
            axes[0, 1].grid(True, alpha=0.3)
        
        # Attitude
        if self.state_data:
            roll = [np.degrees(s.roll) for s in self.state_data]
            pitch = [np.degrees(s.pitch) for s in self.state_data]
            yaw = [np.degrees(s.yaw) for s in self.state_data]
            axes[1, 0].plot(times, roll, 'r-', label='Roll', linewidth=1)
            axes[1, 0].plot(times, pitch, 'g-', label='Pitch', linewidth=1)
            axes[1, 0].plot(times, yaw, 'b-', label='Yaw', linewidth=0.5, alpha=0.7)
            axes[1, 0].set_xlabel('Time (s)')
            axes[1, 0].set_ylabel('Angle (deg)')
            axes[1, 0].set_title('Attitude')
            axes[1, 0].legend()
            axes[1, 0].grid(True, alpha=0.3)
        
        # Acceleration
        if self.imu_data:
            imu_times = [i.timestamp for i in self.imu_data]
            accel_x = [i.accel_x for i in self.imu_data]
            accel_y = [i.accel_y for i in self.imu_data]
            accel_z = [i.accel_z for i in self.imu_data]
            axes[1, 1].plot(imu_times, accel_x, 'r-', label='X', linewidth=0.5, alpha=0.7)
            axes[1, 1].plot(imu_times, accel_y, 'g-', label='Y', linewidth=0.5, alpha=0.7)
            axes[1, 1].plot(imu_times, accel_z, 'b-', label='Z', linewidth=0.5, alpha=0.7)
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Acceleration (m/s²)')
            axes[1, 1].set_title('IMU Acceleration')
            axes[1, 1].legend()
            axes[1, 1].grid(True, alpha=0.3)
        
        # Ground track
        if self.state_data:
            pos_n = [s.pos_n for s in self.state_data]
            pos_e = [s.pos_e for s in self.state_data]
            axes[2, 0].plot(pos_e, pos_n, 'b-', linewidth=1)
            axes[2, 0].plot(0, 0, 'go', markersize=10, label='Launch')
            axes[2, 0].plot(pos_e[-1], pos_n[-1], 'r^', markersize=10, label='Landing')
            axes[2, 0].set_xlabel('East (m)')
            axes[2, 0].set_ylabel('North (m)')
            axes[2, 0].set_title('Ground Track')
            axes[2, 0].legend()
            axes[2, 0].grid(True, alpha=0.3)
            axes[2, 0].axis('equal')
        
        # Barometer
        if self.baro_data:
            baro_times = [b.timestamp for b in self.baro_data]
            altitude = [b.altitude for b in self.baro_data]
            axes[2, 1].plot(baro_times, altitude, 'm-', linewidth=1)
            axes[2, 1].set_xlabel('Time (s)')
            axes[2, 1].set_ylabel('Altitude (m)')
            axes[2, 1].set_title('Barometric Altitude')
            axes[2, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=150)
            print(f"\nPlot saved to {save_path}")
        
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Flight Log Analyzer')
    parser.add_argument('logfile', help='Path to binary log file')
    parser.add_argument('--plot', '-p', action='store_true', 
                        help='Generate plots')
    parser.add_argument('--save', '-s', type=str,
                        help='Save plot to file')
    
    args = parser.parse_args()
    
    analyzer = FlightLogAnalyzer(args.logfile)
    analyzer.parse()
    analyzer.analyze()
    
    if args.plot or args.save:
        analyzer.plot(save_path=args.save)


if __name__ == '__main__':
    main()
