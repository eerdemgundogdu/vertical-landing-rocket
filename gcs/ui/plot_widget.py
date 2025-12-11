"""
Plot Widget - Real-time data plotting using PyQtGraph.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout
from PyQt6.QtCore import Qt
import pyqtgraph as pg
from collections import deque
import time


class PlotWidget(QWidget):
    """Real-time scrolling plot widget."""
    
    def __init__(self, title: str = "", color: str = '#00ff88', max_points: int = 500):
        super().__init__()
        
        self.max_points = max_points
        self.times = deque(maxlen=max_points)
        self.values = deque(maxlen=max_points)
        self.start_time = time.time()
        
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        
        # Configure PyQtGraph
        pg.setConfigOptions(antialias=True, background='#1a1a1a', foreground='#888')
        
        # Create plot widget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setTitle(title, color='#888', size='10pt')
        self.plot_widget.setLabel('bottom', 'Time', 's', color='#666')
        self.plot_widget.setLabel('left', title, color='#666')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setMouseEnabled(x=False, y=False)
        self.plot_widget.setMinimumHeight(120)
        
        # Configure axes
        self.plot_widget.getAxis('bottom').setStyle(tickLength=-5)
        self.plot_widget.getAxis('left').setStyle(tickLength=-5)
        
        # Create curve
        pen = pg.mkPen(color=color, width=2)
        self.curve = self.plot_widget.plot([], [], pen=pen)
        
        layout.addWidget(self.plot_widget)
    
    def add_point(self, time_val: float, value: float):
        """Add a data point to the plot."""
        # Use relative time
        rel_time = time_val - self.start_time
        
        self.times.append(rel_time)
        self.values.append(value)
        
        # Update curve
        if len(self.times) > 1:
            self.curve.setData(list(self.times), list(self.values))
            
            # Auto-scroll to show last 30 seconds
            if len(self.times) > 10:
                x_max = self.times[-1]
                x_min = max(self.times[0], x_max - 30)
                self.plot_widget.setXRange(x_min, x_max, padding=0.02)
    
    def clear(self):
        """Clear all data."""
        self.times.clear()
        self.values.clear()
        self.curve.setData([], [])
        self.start_time = time.time()
