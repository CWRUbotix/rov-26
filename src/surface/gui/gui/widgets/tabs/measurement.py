from dataclasses import dataclass
from enum import IntEnum
from typing import override

from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QHBoxLayout, QPushButton, QVBoxLayout, QWidget

from gui.gui_node import GUINode
from gui.widgets.video_widget import (
    CameraDescription,
    CameraManager,
    CameraType,
    ClickableLabel,
    SwitchableVideoWidget,
)
from rov_msgs.srv import CameraManage
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_default

FRAME_WIDTH = 816
FRAME_HEIGHT = 510
# FRAME_WIDTH = 1280
# FRAME_HEIGHT = 800

class Eye(IntEnum):
    LEFT = 0
    RIGHT = 1

@dataclass
class Point3D:
    x: float
    y: float
    z: float

    @override
    def __str__(self) -> str:
        return f'({round(self.x, 3)}, {round(self.y, 3)}, {round(self.z, 3)})'

class MeasurementTab(QWidget):

    def __init__(self) -> None:
        super().__init__()

        self.point_cloud_manager = CameraManager('manage_luxonis', CameraManage.Request.POINT_CLOUD)
        self.measurement_start_publisher = GUINode().create_publisher(Bool,
                                        'measurement_start', qos_profile_default)

        videos = self.make_videos()

        capture_btn = QPushButton()

        capture_btn.clicked.connect(self.start_point_cloud_capture)

        root_layout = QVBoxLayout()
        root_layout.addWidget(videos)
        root_layout.addWidget(capture_btn)
        self.setLayout(root_layout)

    def make_videos(self) -> QWidget:

        cam_layout = QHBoxLayout()

        # TODO: RESET THIS TO ACTUAL LUXONIS CAM ONCE IT'S WORKING AGAIN
        self.eye_widgets = {
            Eye.LEFT: SwitchableVideoWidget(
                (
                    CameraDescription(
                        CameraType.DEPTH,
                        'rect_left/image_raw',
                        'Stream stopped',
                        FRAME_WIDTH,
                        FRAME_HEIGHT,
                    ),
                    CameraDescription(
                        CameraType.DEPTH,
                        'rect_left/image_raw',
                        'Dual Left Eye',
                        FRAME_WIDTH,
                        FRAME_HEIGHT,
                        CameraManager('manage_luxonis', CameraManage.Request.LUX_LEFT_RECT),
                    ),
                ),
                'switch_rect_stream',
                make_label=lambda: ClickableLabel(self.click_left_signal),
            ),
            Eye.RIGHT: SwitchableVideoWidget(
                (
                    CameraDescription(
                        CameraType.DEPTH,
                        'rect_right/image_raw',
                        'Stream stopped',
                        FRAME_WIDTH,
                        FRAME_HEIGHT,
                    ),
                    CameraDescription(
                        CameraType.DEPTH,
                        'rect_right/image_raw',
                        'Dual Right Eye',
                        FRAME_WIDTH,
                        FRAME_HEIGHT,
                        CameraManager('manage_luxonis', CameraManage.Request.LUX_RIGHT_RECT),
                    ),
                ),
                'switch_rect_stream',
                make_label=lambda: ClickableLabel(self.click_right_signal),
            ),
        }

        for eye_widget in self.eye_widgets.values():
            cam_layout.addWidget(eye_widget)

        coarse_tab = QWidget()
        coarse_tab.setLayout(cam_layout)

        return coarse_tab

    def start_point_cloud_capture(self) -> None:
        self.point_cloud_manager.set_cam_state(on=True)
        self.measurement_start_publisher.publish(Bool(data=True))

