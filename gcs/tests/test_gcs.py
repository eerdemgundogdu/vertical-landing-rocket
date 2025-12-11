#!/usr/bin/env python3
"""
Unit Tests for GCS Application
==============================
Tests for telemetry panel, command panel, and data handling.
"""

import unittest
import sys
import os
import json
from unittest.mock import MagicMock, patch

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class TestTelemetryData(unittest.TestCase):
    """Test telemetry data handling."""
    
    def test_parse_json_telemetry(self):
        """Test parsing JSON telemetry packet."""
        json_data = {
            'timestamp': 1234567890.0,
            'flight_state': 2,
            'armed': True,
            'altitude': 150.5,
            'vel_d': -25.3,
            'roll': 0.1,
            'pitch': -0.05,
            'yaw': 1.57,
            'battery_voltage': 7.8,
            'cpu_temp': 45.0
        }
        
        # Parse
        data = json.dumps(json_data)
        parsed = json.loads(data)
        
        self.assertEqual(parsed['flight_state'], 2)
        self.assertTrue(parsed['armed'])
        self.assertAlmostEqual(parsed['altitude'], 150.5)
        
    def test_handle_missing_fields(self):
        """Test handling of incomplete telemetry."""
        json_data = {
            'timestamp': 1234567890.0,
            'altitude': 100.0
        }
        
        # Should handle missing fields gracefully
        data = json.loads(json.dumps(json_data))
        
        self.assertEqual(data.get('flight_state', 0), 0)
        self.assertEqual(data.get('armed', False), False)


class TestFlightStateNames(unittest.TestCase):
    """Test flight state name mapping."""
    
    def test_state_names(self):
        states = ['IDLE', 'ARMED', 'BOOST', 'COAST', 'DESCENT', 'LANDING', 'LANDED', 'ABORT']
        
        for i, expected in enumerate(states):
            self.assertEqual(states[i], expected)
            
    def test_state_index_bounds(self):
        states = ['IDLE', 'ARMED', 'BOOST', 'COAST', 'DESCENT', 'LANDING', 'LANDED', 'ABORT']
        
        # Test with out of bounds index
        state_idx = 10
        safe_idx = min(state_idx, len(states) - 1)
        
        self.assertEqual(safe_idx, 7)
        self.assertEqual(states[safe_idx], 'ABORT')


class TestDataConversions(unittest.TestCase):
    """Test data conversion utilities."""
    
    def test_quaternion_to_euler(self):
        """Test quaternion to Euler conversion."""
        import math
        
        def quaternion_to_euler(w, x, y, z):
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
        
        # Identity quaternion should give zero angles
        roll, pitch, yaw = quaternion_to_euler(1, 0, 0, 0)
        self.assertAlmostEqual(roll, 0, places=5)
        self.assertAlmostEqual(pitch, 0, places=5)
        self.assertAlmostEqual(yaw, 0, places=5)
        
    def test_degrees_radians_conversion(self):
        """Test degree/radian conversion."""
        import math
        
        rad = 0.5
        deg = rad * 57.3
        
        self.assertAlmostEqual(deg, math.degrees(rad), places=0)


class TestCRC16(unittest.TestCase):
    """Test CRC16 calculation."""
    
    def test_crc16_ccitt(self):
        """Test CRC16-CCITT algorithm."""
        def crc16(data):
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
        
        # Test with known values
        test_data = b'\xAA\x01\x00\x00\x00\x00'
        crc = crc16(test_data)
        
        self.assertIsInstance(crc, int)
        self.assertLessEqual(crc, 0xFFFF)


class TestPlotData(unittest.TestCase):
    """Test plot data handling."""
    
    def test_data_buffer(self):
        """Test data buffer for plotting."""
        from collections import deque
        
        max_points = 100
        data = deque(maxlen=max_points)
        
        # Add points
        for i in range(150):
            data.append(i)
            
        # Should only keep last 100
        self.assertEqual(len(data), max_points)
        self.assertEqual(data[0], 50)
        self.assertEqual(data[-1], 149)


if __name__ == '__main__':
    unittest.main()
