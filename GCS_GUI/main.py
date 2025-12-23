import sys
import os
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QGridLayout,
    QLabel
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPixmap
from PyQt6.QtWidgets import QGraphicsDropShadowEffect

from widgets import VideoWidget, WebWidget, QGCLauncherWidget


class GCSMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("ResQWings | Ground Control Station")
        self.resize(1280, 800)

        # ================= CENTRAL WIDGET =================
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QGridLayout()
        layout.setContentsMargins(16, 12, 16, 16)
        layout.setHorizontalSpacing(12)
        layout.setVerticalSpacing(12)
        central_widget.setLayout(layout)

        # ================= LOGO (TOP LEFT, WITH GLOW) =================
        logo_label = QLabel()
        logo_label.setStyleSheet("background: transparent; border: none;")

        logo_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "assets",
            "NidarLogo.png"
        )

        logo_pixmap = QPixmap(logo_path)
        logo_label.setPixmap(
            logo_pixmap.scaled(
                100, 200,
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.SmoothTransformation
            )
        )

        # ---- Glow effect so BLACK logo is visible ----
        glow = QGraphicsDropShadowEffect()
        glow.setBlurRadius(18)
        glow.setColor(Qt.GlobalColor.white)
        glow.setOffset(0, 0)
        logo_label.setGraphicsEffect(glow)

        # Add logo at top-left
        layout.addWidget(logo_label, 0, 0, alignment=Qt.AlignmentFlag.AlignLeft)

        # ================= TEAM NAME (CENTER, TEXT ONLY) =================
        team_name = QLabel("ResQWings")
        team_name.setAlignment(Qt.AlignmentFlag.AlignCenter)
        team_name.setStyleSheet("""
            background: transparent;
            border: none;
            font-size: 32px;
            font-weight: 800;
            letter-spacing: 2px;
            color: #e0f7ff;
        """)

        layout.addWidget(team_name, 0, 0, 1, 3)

        # ================= COMPONENTS =================
        qgc_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "QGroundControl.AppImage"
        )
        self.qgc_widget = QGCLauncherWidget(app_path=qgc_path)

        self.rpi_connect_drone_2 = WebWidget(
            url="https://www.raspberrypi.com/software/connect/",
            title="RPi Connect - Drone 2"
        )

        self.rpi_connect_drone_1 = WebWidget(
            url="https://www.raspberrypi.com/software/connect/",
            title="RPi Connect - Drone 1"
        )

        self.camera_drone_1 = VideoWidget(
            source=0,
            title="Camera Stream - Drone 1"
        )

        self.camera_drone_2 = VideoWidget(
            source=1,
            title="Camera Stream - Drone 2"
        )

        # ================= GRID =================
        layout.addWidget(self.qgc_widget, 1, 0, 1, 2)
        layout.addWidget(self.rpi_connect_drone_2, 1, 2)

        layout.addWidget(self.rpi_connect_drone_1, 2, 0)
        layout.addWidget(self.camera_drone_1, 2, 1)
        layout.addWidget(self.camera_drone_2, 2, 2)

        # ================= STRETCH =================
        layout.setColumnStretch(0, 1)
        layout.setColumnStretch(1, 1)
        layout.setColumnStretch(2, 1)

        layout.setRowStretch(0, 0)
        layout.setRowStretch(1, 2)
        layout.setRowStretch(2, 1)


def main():
    app = QApplication(sys.argv)

    style_file = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "style.qss"
    )
    if os.path.exists(style_file):
        with open(style_file, "r") as f:
            app.setStyleSheet(f.read())

    window = GCSMainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()