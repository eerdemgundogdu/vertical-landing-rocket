#!/usr/bin/env python3
"""
PID Tuning Tool
===============
Interactive tool for tuning PID controller gains.
Visualizes step response and allows real-time adjustment.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from scipy.integrate import odeint


class PIDSimulator:
    """Simulate PID control of a second-order system."""
    
    def __init__(self):
        # Plant parameters (second-order system: rocket attitude)
        self.J = 0.01    # Moment of inertia
        self.b = 0.001   # Damping
        
        # PID gains
        self.Kp = 5.0
        self.Ki = 0.1
        self.Kd = 0.5
        
        # Setpoint
        self.setpoint = 0.1  # 0.1 rad target
        
    def plant(self, x, t, u):
        """Second-order plant dynamics: J*theta'' + b*theta' = u"""
        theta, theta_dot = x
        theta_ddot = (u - self.b * theta_dot) / self.J
        return [theta_dot, theta_ddot]
    
    def simulate(self, duration=5.0, dt=0.001):
        """Simulate closed-loop response."""
        t = np.arange(0, duration, dt)
        n = len(t)
        
        # States
        theta = np.zeros(n)
        theta_dot = np.zeros(n)
        u = np.zeros(n)
        
        # PID state
        integral = 0
        last_error = 0
        
        for i in range(1, n):
            # Error
            error = self.setpoint - theta[i-1]
            
            # PID
            integral += error * dt
            derivative = (error - last_error) / dt if i > 1 else 0
            
            # Anti-windup
            integral = np.clip(integral, -1, 1)
            
            # Control output
            u[i] = self.Kp * error + self.Ki * integral + self.Kd * derivative
            u[i] = np.clip(u[i], -10, 10)  # Actuator saturation
            
            # Integrate plant
            x0 = [theta[i-1], theta_dot[i-1]]
            x = odeint(self.plant, x0, [0, dt], args=(u[i],))
            theta[i] = x[-1, 0]
            theta_dot[i] = x[-1, 1]
            
            last_error = error
        
        return t, theta, u
    
    def step_info(self, t, y):
        """Calculate step response metrics."""
        target = self.setpoint
        
        # Rise time (10% to 90%)
        y10 = 0.1 * target
        y90 = 0.9 * target
        t10 = t[np.argmax(y >= y10)] if np.any(y >= y10) else t[-1]
        t90 = t[np.argmax(y >= y90)] if np.any(y >= y90) else t[-1]
        rise_time = t90 - t10
        
        # Overshoot
        overshoot = (np.max(y) - target) / target * 100 if target != 0 else 0
        
        # Settling time (within 2%)
        settling_band = 0.02 * target
        settled = np.abs(y - target) <= settling_band
        if np.any(settled):
            # Find last time it exits settling band
            settling_time = t[-1]
            for i in range(len(t)-1, -1, -1):
                if not settled[i]:
                    settling_time = t[i]
                    break
        else:
            settling_time = t[-1]
        
        return {
            'rise_time': rise_time,
            'overshoot': overshoot,
            'settling_time': settling_time
        }


def main():
    """Interactive PID tuning interface."""
    sim = PIDSimulator()
    
    # Initial simulation
    t, y, u = sim.simulate()
    
    # Create figure
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle('Interactive PID Tuner - Vertical Landing Rocket', fontsize=14)
    plt.subplots_adjust(bottom=0.35)
    
    # Response plot
    line1, = ax1.plot(t, y, 'b-', linewidth=2, label='Response')
    ax1.axhline(y=sim.setpoint, color='r', linestyle='--', label='Setpoint')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Angle (rad)')
    ax1.set_title('Step Response')
    ax1.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([0, 5])
    ax1.set_ylim([-0.05, 0.2])
    
    # Control effort plot
    line2, = ax2.plot(t, u, 'g-', linewidth=1, label='Control')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Torque (N·m)')
    ax2.set_title('Control Effort')
    ax2.legend(loc='upper right')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([0, 5])
    
    # Add metrics text
    info = sim.step_info(t, y)
    metrics_text = ax1.text(
        0.02, 0.98, '', transform=ax1.transAxes, 
        verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5)
    )
    
    def update_metrics(info):
        text = (f"Rise Time: {info['rise_time']*1000:.1f} ms\n"
                f"Overshoot: {info['overshoot']:.1f}%\n"
                f"Settling:  {info['settling_time']*1000:.1f} ms")
        metrics_text.set_text(text)
    
    update_metrics(info)
    
    # Sliders
    ax_kp = plt.axes([0.2, 0.25, 0.6, 0.03])
    ax_ki = plt.axes([0.2, 0.20, 0.6, 0.03])
    ax_kd = plt.axes([0.2, 0.15, 0.6, 0.03])
    
    slider_kp = Slider(ax_kp, 'Kp', 0.0, 20.0, valinit=sim.Kp)
    slider_ki = Slider(ax_ki, 'Ki', 0.0, 2.0, valinit=sim.Ki)
    slider_kd = Slider(ax_kd, 'Kd', 0.0, 2.0, valinit=sim.Kd)
    
    def update(val):
        sim.Kp = slider_kp.val
        sim.Ki = slider_ki.val
        sim.Kd = slider_kd.val
        
        t, y, u = sim.simulate()
        line1.set_ydata(y)
        line2.set_ydata(u)
        
        info = sim.step_info(t, y)
        update_metrics(info)
        
        ax2.set_ylim([min(u.min() * 1.1, -0.1), max(u.max() * 1.1, 0.1)])
        
        fig.canvas.draw_idle()
    
    slider_kp.on_changed(update)
    slider_ki.on_changed(update)
    slider_kd.on_changed(update)
    
    # Reset button
    ax_reset = plt.axes([0.8, 0.05, 0.1, 0.04])
    button = Button(ax_reset, 'Reset', hovercolor='0.975')
    
    def reset(event):
        slider_kp.reset()
        slider_ki.reset()
        slider_kd.reset()
    
    button.on_clicked(reset)
    
    plt.show()


if __name__ == '__main__':
    main()
