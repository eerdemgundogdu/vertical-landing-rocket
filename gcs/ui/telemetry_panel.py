"""
Telemetry Panel - Real-time telemetry display with gauges.
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QProgressBar, QGroupBox, QFrame
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


class GaugeWidget(QWidget):
    """Custom gauge widget for displaying values."""
    
    def __init__(self, label: str, unit: str = "", min_val: float = 0, max_val: float = 100):
        super().__init__()
        
        self.min_val = min_val
        self.max_val = max_val
        self.value = 0.0
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(4)
        
        # Label
        self.label = QLabel(label)
        self.label.setStyleSheet("color: #888; font-size: 11px;")
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.label)
        
        # Value display
        self.value_label = QLabel("0.0")
        self.value_label.setFont(QFont("Consolas", 20, QFont.Weight.Bold))
        self.value_label.setStyleSheet("color: #00ff88;")
        self.value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.value_label)
        
        # Unit
        self.unit_label = QLabel(unit)
        self.unit_label.setStyleSheet("color: #666; font-size: 10px;")
        self.unit_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.unit_label)
        
        # Progress bar
        self.progress = QProgressBar()
        self.progress.setMinimum(0)
        self.progress.setMaximum(100)
        self.progress.setTextVisible(False)
        self.progress.setMaximumHeight(8)
        self.progress.setStyleSheet("""
            QProgressBar {
                background: #333;
                border-radius: 4px;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00ff88, stop:1 #00aa55);
                border-radius: 4px;
            }
        """)
        layout.addWidget(self.progress)
    
    def set_value(self, value: float, color: str = None):
        """Update the gauge value."""
        self.value = value
        
        # Update value label
        if abs(value) >= 1000:
            self.value_label.setText(f"{value:.0f}")
        elif abs(value) >= 100:
            self.value_label.setText(f"{value:.1f}")
        else:
            self.value_label.setText(f"{value:.2f}")
        
        # Update progress bar
        if self.max_val > self.min_val:
            percent = int(100 * (value - self.min_val) / (self.max_val - self.min_val))
            percent = max(0, min(100, percent))
            self.progress.setValue(percent)
        
        # Update color
        if color:
            self.value_label.setStyleSheet(f"color: {color};")


class TelemetryPanel(QGroupBox):
    """Panel displaying key telemetry values."""
    
    def __init__(self):
        super().__init__("TELEMETRY")
        
        layout = QGridLayout(self)
        layout.setSpacing(15)
        
        # Altitude gauge
        self.altitude_gauge = GaugeWidget("ALTITUDE", "m", 0, 500)
        layout.addWidget(self.altitude_gauge, 0, 0)
        
        # Velocity gauge  
        self.velocity_gauge = GaugeWidget("VELOCITY", "m/s", -50, 50)
        layout.addWidget(self.velocity_gauge, 0, 1)
        
        # Battery gauge
        self.battery_gauge = GaugeWidget("BATTERY", "V", 6.0, 8.4)
        layout.addWidget(self.battery_gauge, 1, 0)
        
        # Temperature gauge
        self.temp_gauge = GaugeWidget("CPU TEMP", "°C", 20, 80)
        layout.addWidget(self.temp_gauge, 1, 1)
    
    def update_data(self, altitude: float = 0, velocity: float = 0,
                    battery: float = 0, temperature: float = 0):
        """Update all gauge values."""
        self.altitude_gauge.set_value(altitude)
        
        # Color velocity based on direction
        if velocity > 1:
            vel_color = "#00ff88"  # Green = ascending
        elif velocity < -1:
            vel_color = "#ff8800"  # Orange = descending
        else:
            vel_color = "#888888"  # Gray = hover
        self.velocity_gauge.set_value(velocity, vel_color)
        
        # Color battery based on level
        if battery > 7.4:
            bat_color = "#00ff88"
        elif battery > 6.8:
            bat_color = "#ffaa00"
        else:
            bat_color = "#ff4444"
        self.battery_gauge.set_value(battery, bat_color)
        
        # Color temperature
        if temperature < 50:
            temp_color = "#00ff88"
        elif temperature < 70:
            temp_color = "#ffaa00"
        else:
            temp_color = "#ff4444"
        self.temp_gauge.set_value(temperature, temp_color)
