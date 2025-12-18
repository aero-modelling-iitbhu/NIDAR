import sys
import os
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QGridLayout, 
                             QLabel, QVBoxLayout, QFrame)
from PyQt6.QtCore import QFile, QTextStream

from widgets import VideoWidget, WebWidget, QGCLauncherWidget

class GCSMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ground Control Station Console")
        self.resize(1280, 800)

        # Main Central Widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Layout
        # Grid Layout 2 rows, 3 columns
        # Row 0: QGC (Span 2), RPi 2 (Span 1)
        # Row 1: RPi 1, Cam 1, Cam 2
        layout = QGridLayout()
        central_widget.setLayout(layout)

        # Header Title
        # self.header = QLabel("Ground Control Station Console")
        # self.header.setStyleSheet("font-size: 24px; font-weight: bold; padding: 10px;")
        # layout.addWidget(self.header, 0, 0, 1, 3) 
        # Actually header is better as window title or separate top widget. 
        # The prompt image shows it quite prominent. I'll add a top bar.
        
        # Components
        # Path to QGC - Assumed in same directory
        qgc_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "QGroundControl.AppImage")
        self.qgc_widget = QGCLauncherWidget(app_path=qgc_path)
        
        # Web Views (RPi Connect)
        # Placeholders
        self.rpi_connect_drone_2 = WebWidget(url="https://www.raspberrypi.com/software/connect/", title="RPi Connect - Drone 2")
        self.rpi_connect_drone_1 = WebWidget(url="https://www.raspberrypi.com/software/connect/", title="RPi Connect - Drone 1")
        
        # Camera Streams
        # Attempt to open indices 0 and 1. If not available, it handles gracefully.
        self.camera_drone_1 = VideoWidget(source=0, title="Camera Stream - Drone 1")
        self.camera_drone_2 = VideoWidget(source=2, title="Camera Stream - Drone 2") # Using 2 just in case 0/1 collide, or more likely 0 is webcam, 2 is external. 
        # Actually, let's stick to 0 and 1 (or a video file if I had one). default 0.
        # Ideally user can configure this.
        self.camera_drone_2 = VideoWidget(source=1, title="Camera Stream - Drone 2")

        # Styling frames
        # We wrap them in frames to see borders if needed, or just add directly.
        # Widgets have their own dark headers.

        # Add to Layout
        # Grid indices: row, col, rowspan, colspan
        layout.addWidget(self.qgc_widget, 0, 0, 1, 2)
        layout.addWidget(self.rpi_connect_drone_2, 0, 2, 1, 1)
        
        layout.addWidget(self.rpi_connect_drone_1, 1, 0, 1, 1)
        layout.addWidget(self.camera_drone_1, 1, 1, 1, 1)
        layout.addWidget(self.camera_drone_2, 1, 2, 1, 1)

        # Set Column Stretches to ensure 1:1:1 ratio
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)
        
        # Set Row Stretches
        layout.setRowStretch(0, 2) # Top row taller? QGC is big.
        layout.setRowStretch(1, 1)

def main():
    app = QApplication(sys.argv)
    
    # Load Stylesheet
    style_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "style.qss")
    if os.path.exists(style_file):
        with open(style_file, "r") as f:
            app.setStyleSheet(f.read())
            
    window = GCSMainWindow()
    window.show()
    
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
