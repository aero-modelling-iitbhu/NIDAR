import cv2
import sys
import os
import subprocess
import time
from PyQt6.QtWidgets import (QWidget, QLabel, QVBoxLayout, QPushButton, 
                             QFrame, QSizePolicy, QMessageBox)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QUrl, QTimer
from PyQt6.QtGui import QImage, QPixmap
from PyQt6.QtWebEngineWidgets import QWebEngineView

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage)

    def __init__(self, source=0):
        super().__init__()
        self.source = source
        self._run_flag = True

    def run(self):
        # capture from web cam
        cap = cv2.VideoCapture(self.source)
        while self._run_flag:
            ret, cv_img = cap.read()
            if ret:
                rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb_image.shape
                bytes_per_line = ch * w
                convert_to_qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
                p = convert_to_qt_format.scaled(640, 480, Qt.AspectRatioMode.KeepAspectRatio)
                self.change_pixmap_signal.emit(p)
            else:
                # If no camera, sleep a bit to avoid busy loop
                time.sleep(1)
        # shut down capture system
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

class VideoWidget(QWidget):
    def __init__(self, source=0, title="Camera Stream"):
        super().__init__()
        self.setWindowTitle(title)
        self.disply_width = 640
        self.display_height = 480
        
        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("font-weight: bold; color: white; background-color: #333; padding: 5px;")

        self.image_label = QLabel(self)
        self.image_label.resize(self.disply_width, self.display_height)
        self.image_label.setStyleSheet("background-color: black;")
        self.image_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.image_label.setText("No Signal")

        layout = QVBoxLayout()
        layout.addWidget(self.title_label)
        layout.addWidget(self.image_label)
        layout.setContentsMargins(0,0,0,0)
        self.setLayout(layout)

        self.source = source
        self.thread = VideoThread(source)
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

    def update_image(self, qt_img):
        self.image_label.setPixmap(QPixmap.fromImage(qt_img))

    def closeEvent(self, event):
        self.thread.stop()
        event.accept()

class WebWidget(QWidget):
    def __init__(self, url="https://google.com", title="Web View"):
        super().__init__()
        
        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("font-weight: bold; color: white; background-color: #333; padding: 5px;")

        self.browser = QWebEngineView()
        self.browser.setUrl(QUrl(url))

        layout = QVBoxLayout()
        layout.addWidget(self.title_label)
        layout.addWidget(self.browser)
        layout.setContentsMargins(0,0,0,0)
        self.setLayout(layout)

class QGCLauncherWidget(QWidget):
    def __init__(self, app_path="./QGroundControl.AppImage"):
        super().__init__()
        self.app_path = app_path
        self.process = None
        self.embedded_window = None

        self.title_label = QLabel("QGroundControl")
        self.title_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.title_label.setStyleSheet("font-weight: bold; color: white; background-color: #222; font-size: 16px;")

        self.status_label = QLabel("Ready to Launch")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet("color: #aaa;")

        self.launch_btn = QPushButton("Launch QGroundControl")
        self.launch_btn.clicked.connect(self.launch_qgc)
        self.launch_btn.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50; 
                color: white; 
                padding: 10px; 
                border-radius: 5px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)

        self.layout = QVBoxLayout()
        self.layout.addStretch()
        self.layout.addWidget(self.title_label)
        self.layout.addWidget(self.status_label)
        self.layout.addWidget(self.launch_btn)
        self.layout.addStretch()
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(self.layout)
        self.setStyleSheet("background-color: #1e1e1e;")

    def launch_qgc(self):
        # Resolve absolute path
        if not os.path.exists(self.app_path):
             if os.path.exists(os.path.join(os.getcwd(), "QGroundControl.AppImage")):
                 self.app_path = os.path.join(os.getcwd(), "QGroundControl.AppImage")
             elif os.path.exists(os.path.join(os.path.dirname(__file__), "QGroundControl.AppImage")):
                 self.app_path = os.path.join(os.path.dirname(__file__), "QGroundControl.AppImage")
             else:
                 self.status_label.setText(f"Error: Not found at {self.app_path}")
                 return

        try:
            os.chmod(self.app_path, 0o755)
            self.process = subprocess.Popen([self.app_path])
            self.status_label.setText("Launching... Please wait.")
            self.launch_btn.setEnabled(False)
            
            # Start checking for the window
            QTimer.singleShot(2000, self.check_for_window)
        except Exception as e:
            self.status_label.setText(f"Failed: {str(e)}")
            self.launch_btn.setEnabled(True)

    def check_for_window(self):
        wid = self.find_qgc_wid()
        if wid:
            self.embed_window(wid)
        else:
            # Keep checking for up to 30 seconds
            QTimer.singleShot(1000, self.check_for_window)

    def find_qgc_wid(self):
        try:
            print("DEBUG: Checking for QGC window...")
            # querying xwininfo tree
            output = subprocess.check_output(['xwininfo', '-root', '-tree']).decode('utf-8', errors='ignore')
            for line in output.splitlines():
                # Flexible matching
                low_line = line.lower()
                if "qgroundcontrol" in low_line:
                    print(f"DEBUG: Found candidate line: {line.strip()}")
                    # Line format usually: "  0x4400001 "QGroundControl": ..."
                    parts = line.strip().split()
                    if parts:
                        wid_str = parts[0]
                        if wid_str.startswith('0x'):
                            # Verify if it really looks like a window id
                            try:
                                wid = int(wid_str, 16)
                                print(f"DEBUG: Returning WID: {hex(wid)}")
                                return wid
                            except ValueError:
                                continue
        except Exception as e:
            print(f"DEBUG: Error finding window: {e}")
        return None

    def embed_window(self, wid):
        try:
            from PyQt6.QtGui import QWindow
            
            window = QWindow.fromWinId(wid)
            self.embedded_window = QWidget.createWindowContainer(window)
            
            # Remove placeholder widgets
            self.title_label.hide()
            self.status_label.hide()
            self.launch_btn.hide()
            
            # Add embedded window
            self.layout.addWidget(self.embedded_window)
            self.layout.setStretch(0, 0) # Reset stretches
            self.layout.setStretch(1, 1) # Make sure embedded window expands
            
            self.status_label.setText("Embedded")
            print(f"Embedded window {hex(wid)}")
        except Exception as e:
            self.status_label.setText(f"Embedding failed: {e}")
            self.title_label.show()
            self.status_label.show()

