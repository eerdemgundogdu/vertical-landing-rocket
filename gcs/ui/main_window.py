"""
Main Window - Ground Control Station
=====================================
Professional dark-themed GCS interface with dockable panels.
"""

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QDockWidget, QStatusBar, QToolBar, QLabel, QPushButton,
    QGroupBox, QProgressBar, QTextEdit, QSplitter, QFrame
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QAction, QFont, QColor, QPalette

import socket
import json
import threading
from dataclasses import dataclass
from typing import Optional

from .telemetry_panel import TelemetryPanel
from .plot_widget import PlotWidget
from .attitude_indicator import AttitudeIndicator
from .command_panel import CommandPanel


@dataclass
class TelemetryData:
    """Telemetry data structure."""
    timestamp: float = 0.0
    flight_state: int = 0
    armed: bool = False
    pos_n: float = 0.0
    pos_e: float = 0.0
    pos_d: float = 0.0
    altitude: float = 0.0
    vel_n: float = 0.0
    vel_e: float = 0.0
    vel_d: float = 0.0
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    battery_voltage: float = 0.0
    cpu_temp: float = 0.0
    sensor_status: int = 0


class MainWindow(QMainWindow):
    """Main GCS window with dockable panels."""
    
    telemetry_received = pyqtSignal(dict)
    
    def __init__(self):
        super().__init__()
        
        self.setWindowTitle("🚀 Vertical Landing Rocket - Ground Control Station")
        self.setMinimumSize(1400, 900)
        
        # Telemetry state
        self.telemetry = TelemetryData()
        self.connected = False
        self.udp_thread: Optional[threading.Thread] = None
        self.running = False
        
        # Setup UI
        self.setup_ui()
        self.setup_menu()
        self.setup_toolbar()
        self.setup_statusbar()
        
        # Connect signal
        self.telemetry_received.connect(self.update_telemetry)
        
        # Start UDP listener
        self.start_udp_listener()
        
        # Update timer (60 FPS)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(16)  # ~60 FPS
    
    def setup_ui(self):
        """Setup the main UI layout."""
        # Central widget with main content
        central = QWidget()
        self.setCentralWidget(central)
        
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)
        
        # Left side - Attitude and Telemetry
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)
        
        # Attitude Indicator
        attitude_group = QGroupBox("ATTITUDE")
        attitude_layout = QVBoxLayout(attitude_group)
        self.attitude_indicator = AttitudeIndicator()
        self.attitude_indicator.setMinimumSize(300, 300)
        attitude_layout.addWidget(self.attitude_indicator)
        left_layout.addWidget(attitude_group)
        
        # Telemetry Panel
        self.telemetry_panel = TelemetryPanel()
        left_layout.addWidget(self.telemetry_panel)
        
        main_layout.addWidget(left_panel)
        
        # Center - Plots
        center_panel = QWidget()
        center_layout = QVBoxLayout(center_panel)
        center_layout.setContentsMargins(0, 0, 0, 0)
        center_layout.setSpacing(10)
        
        plots_group = QGroupBox("FLIGHT DATA")
        plots_layout = QVBoxLayout(plots_group)
        
        # Altitude plot
        self.altitude_plot = PlotWidget("Altitude (m)", color='#00ff88')
        plots_layout.addWidget(self.altitude_plot)
        
        # Velocity plot
        self.velocity_plot = PlotWidget("Velocity (m/s)", color='#ff8800')
        plots_layout.addWidget(self.velocity_plot)
        
        # Attitude plot
        self.attitude_plot = PlotWidget("Attitude (deg)", color='#00aaff')
        plots_layout.addWidget(self.attitude_plot)
        
        center_layout.addWidget(plots_group)
        main_layout.addWidget(center_panel, stretch=2)
        
        # Right side - Command Panel and Log
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)
        
        # Command Panel
        self.command_panel = CommandPanel()
        self.command_panel.arm_signal.connect(self.on_arm)
        self.command_panel.disarm_signal.connect(self.on_disarm)
        self.command_panel.abort_signal.connect(self.on_abort)
        right_layout.addWidget(self.command_panel)
        
        # Mission Log
        log_group = QGroupBox("MISSION LOG")
        log_layout = QVBoxLayout(log_group)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Consolas", 9))
        self.log_text.setMaximumHeight(200)
        log_layout.addWidget(self.log_text)
        right_layout.addWidget(log_group)
        
        right_layout.addStretch()
        main_layout.addWidget(right_panel)
    
    def setup_menu(self):
        """Setup menu bar."""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("File")
        
        connect_action = QAction("Connect", self)
        connect_action.triggered.connect(self.start_udp_listener)
        file_menu.addAction(connect_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # View menu
        view_menu = menubar.addMenu("View")
        
        reset_view = QAction("Reset Layout", self)
        view_menu.addAction(reset_view)
    
    def setup_toolbar(self):
        """Setup toolbar."""
        toolbar = QToolBar("Main Toolbar")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        
        # Connection status
        self.conn_label = QLabel(" ● DISCONNECTED ")
        self.conn_label.setStyleSheet("color: #ff4444; font-weight: bold;")
        toolbar.addWidget(self.conn_label)
        
        toolbar.addSeparator()
        
        # Flight state
        self.state_label = QLabel(" IDLE ")
        self.state_label.setStyleSheet(
            "background: #333; color: #00ff88; padding: 4px 12px; "
            "border-radius: 4px; font-weight: bold; font-size: 14px;"
        )
        toolbar.addWidget(self.state_label)
        
        toolbar.addSeparator()
        
        # Timer
        self.timer_label = QLabel(" T+ 00:00.0 ")
        self.timer_label.setStyleSheet("font-family: monospace; font-size: 16px;")
        toolbar.addWidget(self.timer_label)
    
    def setup_statusbar(self):
        """Setup status bar."""
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)
        
        self.statusbar.showMessage("Ready - Waiting for telemetry...")
    
    def start_udp_listener(self):
        """Start UDP listener thread."""
        if self.running:
            return
        
        self.running = True
        self.udp_thread = threading.Thread(target=self.udp_receive_loop, daemon=True)
        self.udp_thread.start()
        
        self.log("UDP listener started on port 14550")
    
    def udp_receive_loop(self):
        """UDP receive loop (runs in background thread)."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', 14550))
            sock.settimeout(1.0)
            
            while self.running:
                try:
                    data, addr = sock.recvfrom(1024)
                    try:
                        telemetry = json.loads(data.decode('utf-8'))
                        self.telemetry_received.emit(telemetry)
                    except json.JSONDecodeError:
                        pass
                except socket.timeout:
                    pass
        except Exception as e:
            print(f"UDP error: {e}")
        finally:
            sock.close()
    
    def update_telemetry(self, data: dict):
        """Update telemetry from received data."""
        self.connected = True
        
        # Update telemetry struct
        self.telemetry.timestamp = data.get('timestamp', 0)
        self.telemetry.flight_state = data.get('flight_state', 0)
        self.telemetry.armed = data.get('armed', False)
        self.telemetry.altitude = data.get('altitude', 0)
        self.telemetry.vel_d = data.get('vel_d', 0)
        self.telemetry.roll = data.get('roll', 0)
        self.telemetry.pitch = data.get('pitch', 0)
        self.telemetry.yaw = data.get('yaw', 0)
        self.telemetry.battery_voltage = data.get('battery_voltage', 0)
        self.telemetry.cpu_temp = data.get('cpu_temp', 0)
    
    def update_display(self):
        """Update display at 60 FPS."""
        # Connection status
        if self.connected:
            self.conn_label.setText(" ● CONNECTED ")
            self.conn_label.setStyleSheet("color: #00ff88; font-weight: bold;")
        else:
            self.conn_label.setText(" ● DISCONNECTED ")
            self.conn_label.setStyleSheet("color: #ff4444; font-weight: bold;")
        
        # Flight state
        states = ['IDLE', 'ARMED', 'BOOST', 'COAST', 'DESCENT', 'LANDING', 'LANDED', 'ABORT']
        state_idx = min(self.telemetry.flight_state, len(states) - 1)
        self.state_label.setText(f" {states[state_idx]} ")
        
        # Update attitude indicator
        self.attitude_indicator.set_attitude(
            self.telemetry.roll * 57.3,  # Convert to degrees
            self.telemetry.pitch * 57.3,
            self.telemetry.yaw * 57.3
        )
        
        # Update telemetry panel
        self.telemetry_panel.update_data(
            altitude=self.telemetry.altitude,
            velocity=-self.telemetry.vel_d,  # Convert to climb rate
            battery=self.telemetry.battery_voltage,
            temperature=self.telemetry.cpu_temp
        )
        
        # Update plots
        import time
        t = time.time()
        self.altitude_plot.add_point(t, self.telemetry.altitude)
        self.velocity_plot.add_point(t, -self.telemetry.vel_d)
        self.attitude_plot.add_point(t, self.telemetry.pitch * 57.3)
    
    def log(self, message: str):
        """Add message to mission log."""
        import datetime
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f"[{timestamp}] {message}")
    
    def on_arm(self):
        """Handle arm command."""
        self.log("ARM command sent")
    
    def on_disarm(self):
        """Handle disarm command."""
        self.log("DISARM command sent")
    
    def on_abort(self):
        """Handle abort command."""
        self.log("!!! ABORT COMMAND SENT !!!")
    
    def closeEvent(self, event):
        """Clean up on close."""
        self.running = False
        if self.udp_thread:
            self.udp_thread.join(timeout=1.0)
        event.accept()
