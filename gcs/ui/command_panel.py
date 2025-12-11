"""
Command Panel - Manual control and command interface.
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QGroupBox, QLabel, QSlider, QSpinBox
)
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtGui import QFont


class CommandPanel(QGroupBox):
    """Panel for sending commands to the rocket."""
    
    arm_signal = pyqtSignal()
    disarm_signal = pyqtSignal()
    abort_signal = pyqtSignal()
    
    def __init__(self):
        super().__init__("COMMAND")
        
        layout = QVBoxLayout(self)
        layout.setSpacing(15)
        
        # ARM/DISARM buttons
        arm_layout = QHBoxLayout()
        
        self.arm_btn = QPushButton("ARM")
        self.arm_btn.setMinimumHeight(50)
        self.arm_btn.setStyleSheet("""
            QPushButton {
                background: #004400;
                color: #00ff88;
                font-size: 16px;
                font-weight: bold;
                border: 2px solid #00ff88;
                border-radius: 8px;
            }
            QPushButton:hover {
                background: #006600;
            }
            QPushButton:pressed {
                background: #008800;
            }
        """)
        self.arm_btn.clicked.connect(self.on_arm)
        arm_layout.addWidget(self.arm_btn)
        
        self.disarm_btn = QPushButton("DISARM")
        self.disarm_btn.setMinimumHeight(50)
        self.disarm_btn.setStyleSheet("""
            QPushButton {
                background: #444400;
                color: #ffaa00;
                font-size: 16px;
                font-weight: bold;
                border: 2px solid #ffaa00;
                border-radius: 8px;
            }
            QPushButton:hover {
                background: #666600;
            }
            QPushButton:pressed {
                background: #888800;
            }
        """)
        self.disarm_btn.clicked.connect(self.on_disarm)
        arm_layout.addWidget(self.disarm_btn)
        
        layout.addLayout(arm_layout)
        
        # ABORT button
        self.abort_btn = QPushButton("⚠ ABORT ⚠")
        self.abort_btn.setMinimumHeight(60)
        self.abort_btn.setStyleSheet("""
            QPushButton {
                background: #440000;
                color: #ff4444;
                font-size: 20px;
                font-weight: bold;
                border: 3px solid #ff4444;
                border-radius: 10px;
            }
            QPushButton:hover {
                background: #660000;
            }
            QPushButton:pressed {
                background: #880000;
            }
        """)
        self.abort_btn.clicked.connect(self.on_abort)
        layout.addWidget(self.abort_btn)
        
        # Calibrate button
        self.calibrate_btn = QPushButton("Calibrate Sensors")
        self.calibrate_btn.setStyleSheet("""
            QPushButton {
                background: #333;
                color: #888;
                padding: 8px;
                border: 1px solid #555;
                border-radius: 4px;
            }
            QPushButton:hover {
                background: #444;
                color: #aaa;
            }
        """)
        layout.addWidget(self.calibrate_btn)
        
        # Landing target coordinates
        target_group = QGroupBox("Landing Target")
        target_layout = QVBoxLayout(target_group)
        
        # North coordinate
        north_layout = QHBoxLayout()
        north_layout.addWidget(QLabel("North:"))
        self.north_spin = QSpinBox()
        self.north_spin.setRange(-100, 100)
        self.north_spin.setSuffix(" m")
        north_layout.addWidget(self.north_spin)
        target_layout.addLayout(north_layout)
        
        # East coordinate
        east_layout = QHBoxLayout()
        east_layout.addWidget(QLabel("East:"))
        self.east_spin = QSpinBox()
        self.east_spin.setRange(-100, 100)
        self.east_spin.setSuffix(" m")
        east_layout.addWidget(self.east_spin)
        target_layout.addLayout(east_layout)
        
        layout.addWidget(target_group)
    
    def on_arm(self):
        """Handle ARM button click."""
        self.arm_signal.emit()
    
    def on_disarm(self):
        """Handle DISARM button click."""
        self.disarm_signal.emit()
    
    def on_abort(self):
        """Handle ABORT button click."""
        self.abort_signal.emit()
