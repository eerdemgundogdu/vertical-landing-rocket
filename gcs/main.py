#!/usr/bin/env python3
"""
Ground Control Station - Main Entry Point
==========================================
PyQt6-based professional GCS for vertical landing rocket.
"""

import sys
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt
from ui.main_window import MainWindow


def main():
    """Launch the Ground Control Station application."""
    # Enable high DPI scaling
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    
    app = QApplication(sys.argv)
    app.setApplicationName("Vertical Landing Rocket GCS")
    app.setApplicationVersion("1.0.0")
    app.setOrganizationName("VLR Project")
    
    # Load stylesheet
    try:
        with open("styles/dark_theme.qss", "r") as f:
            app.setStyleSheet(f.read())
    except FileNotFoundError:
        pass  # Use default style
    
    # Create and show main window
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
