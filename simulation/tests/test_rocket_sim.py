#!/usr/bin/env python3
"""
Unit Tests for Rocket Simulation
================================
Tests for flight dynamics, controller, and trajectory planning.
"""

import unittest
import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rocket_sim import RocketSimulator, RocketParams, Environment, RocketState


class TestRocketParams(unittest.TestCase):
    """Test rocket physical parameters."""
    
    def test_default_params(self):
        params = RocketParams()
        self.assertEqual(params.mass_wet, 1.5)
        self.assertEqual(params.mass_dry, 1.2)
        self.assertGreater(params.Ixx, 0)
        self.assertGreater(params.Iyy, 0)
        
    def test_area_calculation(self):
        params = RocketParams()
        expected_area = np.pi * (params.diameter / 2) ** 2
        self.assertAlmostEqual(params.area, expected_area)


class TestEnvironment(unittest.TestCase):
    """Test environmental model."""
    
    def test_sea_level_density(self):
        env = Environment()
        self.assertAlmostEqual(env.air_density(0), env.rho_sl)
        
    def test_density_decreases_with_altitude(self):
        env = Environment()
        rho_0 = env.air_density(0)
        rho_1000 = env.air_density(1000)
        rho_10000 = env.air_density(10000)
        
        self.assertGreater(rho_0, rho_1000)
        self.assertGreater(rho_1000, rho_10000)


class TestRocketState(unittest.TestCase):
    """Test rocket state vector."""
    
    def test_default_state(self):
        state = RocketState()
        self.assertEqual(state.altitude, 0)
        self.assertEqual(state.qw, 1)  # Identity quaternion
        
    def test_to_array_and_back(self):
        state = RocketState(pos_n=10, vel_d=-5, qw=0.707, qx=0.707)
        arr = state.to_array()
        restored = RocketState.from_array(arr)
        
        self.assertAlmostEqual(restored.pos_n, 10)
        self.assertAlmostEqual(restored.vel_d, -5)
        self.assertAlmostEqual(restored.qw, 0.707)
        
    def test_euler_angles(self):
        state = RocketState()
        roll, pitch, yaw = state.get_euler()
        
        self.assertAlmostEqual(roll, 0)
        self.assertAlmostEqual(pitch, 0)
        self.assertAlmostEqual(yaw, 0)


class TestSimulator(unittest.TestCase):
    """Test flight simulator."""
    
    def setUp(self):
        self.sim = RocketSimulator()
        
    def test_initial_state(self):
        self.assertEqual(self.sim.state.altitude, 0)
        self.assertFalse(self.sim.launched)
        self.assertFalse(self.sim.landed)
        
    def test_thrust_before_launch(self):
        thrust = self.sim.thrust(0)
        self.assertEqual(thrust, 0)
        
    def test_thrust_during_burn(self):
        self.sim.launched = True
        thrust = self.sim.thrust(0.5)
        self.assertGreater(thrust, 0)
        
    def test_thrust_after_burnout(self):
        self.sim.launched = True
        thrust = self.sim.thrust(self.sim.params.burn_time + 1)
        self.assertEqual(thrust, 0)
        
    def test_mass_decreases_during_burn(self):
        self.sim.launched = True
        m_start = self.sim.mass(0)
        m_mid = self.sim.mass(self.sim.params.burn_time / 2)
        m_end = self.sim.mass(self.sim.params.burn_time)
        
        self.assertGreater(m_start, m_mid)
        self.assertGreater(m_mid, m_end)
        self.assertAlmostEqual(m_end, self.sim.params.mass_dry)
        
    def test_step_integration(self):
        self.sim.launch()
        initial_alt = self.sim.state.altitude
        
        # Step for 100ms
        for _ in range(100):
            self.sim.step(0.001)
            
        self.assertGreater(self.sim.state.altitude, initial_alt)
        
    def test_gimbal_limits(self):
        self.sim.set_gimbal(20, 20)  # Beyond limit
        max_rad = np.radians(self.sim.params.max_gimbal)
        
        self.assertLessEqual(abs(self.sim.gimbal_x), max_rad)
        self.assertLessEqual(abs(self.sim.gimbal_y), max_rad)


class TestController(unittest.TestCase):
    """Test simple controller."""
    
    def test_controller_stabilizes(self):
        sim = RocketSimulator()
        sim.launch()
        
        # Set initial disturbance
        sim.state.qx = 0.1
        
        for _ in range(1000):
            sim.simple_controller()
            sim.step(0.001)
            
        roll, pitch, yaw = sim.state.get_euler()
        
        # Should stabilize to near vertical
        self.assertLess(abs(roll), 0.5)
        self.assertLess(abs(pitch), 0.5)


class TestPhysics(unittest.TestCase):
    """Test physics calculations."""
    
    def test_gravity_acts_downward(self):
        sim = RocketSimulator()
        
        # Step without motor
        for _ in range(100):
            sim.step(0.001)
            
        # Should fall (vel_d positive in NED)
        self.assertGreater(sim.state.vel_d, 0)
        
    def test_ground_contact(self):
        sim = RocketSimulator()
        sim.state.pos_d = 0.1  # Start slightly below ground
        sim.state.vel_d = 10  # Moving down
        
        for _ in range(100):
            sim.step(0.001)
            
        # Should not go below ground
        self.assertGreaterEqual(sim.state.altitude, -0.1)


if __name__ == '__main__':
    unittest.main()
