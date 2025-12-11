"""
Attitude Indicator - 3D attitude display widget.
"""

from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QFont, QLinearGradient
import math


class AttitudeIndicator(QWidget):
    """Attitude indicator (artificial horizon) widget."""
    
    def __init__(self):
        super().__init__()
        
        self.roll = 0.0   # degrees
        self.pitch = 0.0  # degrees
        self.yaw = 0.0    # degrees
        
        self.setMinimumSize(200, 200)
    
    def set_attitude(self, roll: float, pitch: float, yaw: float):
        """Update attitude values (in degrees)."""
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.update()  # Trigger repaint
    
    def paintEvent(self, event):
        """Paint the attitude indicator."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Get widget dimensions
        w = self.width()
        h = self.height()
        size = min(w, h) - 20
        cx = w // 2
        cy = h // 2
        
        # Background
        painter.fillRect(self.rect(), QColor("#1a1a1a"))
        
        # Save state and translate to center
        painter.save()
        painter.translate(cx, cy)
        
        # Draw horizon (rotated by roll)
        painter.save()
        painter.rotate(-self.roll)
        
        # Calculate pitch offset (pixels per degree)
        pitch_scale = size / 60  # 60 degrees visible
        pitch_offset = self.pitch * pitch_scale
        
        # Sky (blue gradient)
        sky = QLinearGradient(0, -size, 0, 0)
        sky.setColorAt(0, QColor("#0066aa"))
        sky.setColorAt(1, QColor("#0088cc"))
        painter.setBrush(QBrush(sky))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRect(int(-size), int(-size + pitch_offset), int(size * 2), int(size))
        
        # Ground (brown gradient)
        ground = QLinearGradient(0, 0, 0, size)
        ground.setColorAt(0, QColor("#885522"))
        ground.setColorAt(1, QColor("#553311"))
        painter.setBrush(QBrush(ground))
        painter.drawRect(int(-size), int(pitch_offset), int(size * 2), int(size))
        
        # Horizon line
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawLine(int(-size), int(pitch_offset), int(size), int(pitch_offset))
        
        # Pitch ladder
        painter.setPen(QPen(QColor("#ffffff"), 1))
        painter.setFont(QFont("Arial", 8))
        
        for deg in range(-30, 31, 10):
            if deg == 0:
                continue
            y = pitch_offset - deg * pitch_scale
            if abs(y) < size * 0.4:
                line_len = 30 if deg % 20 == 0 else 15
                painter.drawLine(int(-line_len), int(y), int(line_len), int(y))
                if deg % 20 == 0:
                    painter.drawText(int(line_len + 5), int(y + 4), str(abs(deg)))
                    painter.drawText(int(-line_len - 20), int(y + 4), str(abs(deg)))
        
        painter.restore()  # Restore from roll rotation
        
        # Draw fixed aircraft symbol
        painter.setPen(QPen(QColor("#ffff00"), 3))
        # Wings
        painter.drawLine(-50, 0, -15, 0)
        painter.drawLine(15, 0, 50, 0)
        # Center dot
        painter.setBrush(QBrush(QColor("#ffff00")))
        painter.drawEllipse(-5, -5, 10, 10)
        # Tail
        painter.drawLine(0, -5, 0, -25)
        
        # Draw roll indicator arc
        painter.setPen(QPen(QColor("#ffffff"), 2))
        roll_radius = size * 0.45
        painter.drawArc(
            QRectF(-roll_radius, -roll_radius, roll_radius * 2, roll_radius * 2),
            int((90 + 60) * 16), int(-120 * 16)
        )
        
        # Roll tick marks
        for angle in [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]:
            rad = math.radians(angle - 90)
            x1 = roll_radius * math.cos(rad)
            y1 = roll_radius * math.sin(rad)
            tick_len = 15 if angle % 30 == 0 else 8
            x2 = (roll_radius - tick_len) * math.cos(rad)
            y2 = (roll_radius - tick_len) * math.sin(rad)
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
        
        # Roll pointer (rotates with roll)
        painter.save()
        painter.rotate(-self.roll)
        painter.setBrush(QBrush(QColor("#ffff00")))
        painter.setPen(Qt.PenStyle.NoPen)
        points = [
            (0, int(-roll_radius + 5)),
            (-8, int(-roll_radius - 10)),
            (8, int(-roll_radius - 10))
        ]
        from PyQt6.QtGui import QPolygon
        from PyQt6.QtCore import QPoint
        polygon = QPolygon([QPoint(p[0], p[1]) for p in points])
        painter.drawPolygon(polygon)
        painter.restore()
        
        painter.restore()  # Restore from center translation
        
        # Draw heading indicator
        painter.setPen(QPen(QColor("#00ff88"), 1))
        painter.setFont(QFont("Consolas", 10))
        heading_str = f"HDG {self.yaw:03.0f}°"
        painter.drawText(cx - 35, h - 10, heading_str)
        
        # Draw roll/pitch values
        painter.setPen(QPen(QColor("#888888"), 1))
        painter.setFont(QFont("Consolas", 9))
        painter.drawText(10, 20, f"R: {self.roll:+6.1f}°")
        painter.drawText(10, 35, f"P: {self.pitch:+6.1f}°")
