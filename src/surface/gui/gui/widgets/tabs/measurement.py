from enum import IntEnum

from PyQt6.QtCore import pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QGroupBox, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget
from rclpy.qos import qos_profile_default
from std_msgs.msg import Bool

from gui.gui_node import GUINode
from gui.widgets.video_widget import (
    CameraDescription,
    CameraManager,
    CameraType,
    SwitchableVideoWidget,
)
from rov_msgs.msg import Measurement
from rov_msgs.srv import CameraManage

# FRAME_WIDTH = 816
# FRAME_HEIGHT = 510
FRAME_WIDTH = 721
FRAME_HEIGHT = 541
# FRAME_WIDTH = 1280
# FRAME_HEIGHT = 800

class Eye(IntEnum):
    LEFT = 0
    RIGHT = 1

POINT_LABEL_TEXT = 'Point: '
DISTANCE_LABEL_TEXT = 'DISTANCE: '

class MeasurementTab(QWidget):
    signal = pyqtSignal(Measurement)

    def __init__(self) -> None:
        super().__init__()

        self.point_cloud_manager = CameraManager('manage_luxonis', CameraManage.Request.POINT_CLOUD)
        self.measurement_start_publisher = GUINode().create_publisher(Bool,
                'measurement_start', qos_profile_default)

        self.signal.connect(self.measurement_callback)

        GUINode().create_signal_subscription(Measurement,
                'measurement_calculation', self.signal)

        video_group = QGroupBox('Videos')

        videos = self.make_videos()

        capture_btn = QPushButton('Capture Point Cloud')

        capture_btn.clicked.connect(self.start_point_cloud_capture)

        video_layout = QVBoxLayout()
        video_layout.addWidget(videos)
        video_layout.addWidget(capture_btn)
        video_group.setLayout(video_layout)

        results_group = QGroupBox('Results')
        self.point1_label = QLabel(POINT_LABEL_TEXT)
        self.point2_label = QLabel(POINT_LABEL_TEXT)
        self.distance_label = QLabel(DISTANCE_LABEL_TEXT)

        results_layout = QVBoxLayout()
        results_layout.addWidget(self.point1_label)
        results_layout.addWidget(self.point2_label)
        results_layout.addWidget(self.distance_label)

        results_group.setLayout(results_layout)

        root_layout = QVBoxLayout()
        root_layout.addWidget(video_group)
        root_layout.addWidget(results_group)
        self.setLayout(root_layout)

    def make_videos(self) -> QWidget:

        cam_layout = QHBoxLayout()

        # self.eye_widgets = {
        #     Eye.LEFT: SwitchableVideoWidget(
        #         (
        #             CameraDescription(
        #                 CameraType.DEPTH,
        #                 'rect_left/image_raw',
        #                 'Stream stopped',
        #                 FRAME_WIDTH,
        #                 FRAME_HEIGHT,
        #             ),
        #             CameraDescription(
        #                 CameraType.DEPTH,
        #                 'rect_left/image_raw',
        #                 'Dual Left Eye',
        #                 FRAME_WIDTH,
        #                 FRAME_HEIGHT,
        #                 CameraManager('manage_luxonis', CameraManage.Request.LUX_LEFT_RECT),
        #             ),
        #         ),
        #         'switch_rect_stream',
        #     ),
        #     Eye.RIGHT: SwitchableVideoWidget(
        #         (
        #             CameraDescription(
        #                 CameraType.DEPTH,
        #                 'rect_right/image_raw',
        #                 'Stream stopped',
        #                 FRAME_WIDTH,
        #                 FRAME_HEIGHT,
        #             ),
        #             CameraDescription(
        #                 CameraType.DEPTH,
        #                 'rect_right/image_raw',
        #                 'Dual Right Eye',
        #                 FRAME_WIDTH,
        #                 FRAME_HEIGHT,
        #                 CameraManager('manage_luxonis', CameraManage.Request.LUX_RIGHT_RECT),
        #             ),
        #         ),
        #         'switch_rect_stream',
        #     ),
        # }

        self.eye_widgets = {
            Eye.LEFT: SwitchableVideoWidget(
                (
                    CameraDescription(
                        CameraType.DEPTH,
                        'lux_raw/image_raw',
                        'Stream stopped',
                        FRAME_WIDTH,
                        FRAME_HEIGHT,
                    ),
                    CameraDescription(
                        CameraType.DEPTH,
                        'lux_raw/image_raw',
                        'Dual Left Eye',
                        FRAME_WIDTH,
                        FRAME_HEIGHT,
                        CameraManager('manage_luxonis', CameraManage.Request.LUX_LEFT),
                    ),
                ),
                'switch_measure_stream',
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

    @pyqtSlot(Measurement)
    def measurement_callback(self, msg: Measurement) -> None:

        self.point1_label.setText(
            f'{POINT_LABEL_TEXT}({msg.point1.x}, {msg.point1.y}, {msg.point1.z})'
        )
        self.point2_label.setText(
            f'{POINT_LABEL_TEXT}({msg.point2.x}, {msg.point2.y}, {msg.point2.z})'
        )
        self.distance_label.setText(f'{DISTANCE_LABEL_TEXT}{msg.distance} cm')
