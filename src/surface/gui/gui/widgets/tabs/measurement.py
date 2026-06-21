from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from enum import Enum, IntEnum
from math import atan, sqrt, tan
from typing import Generic, TypeGuard, TypeVar, override

from PyQt6.QtCore import QRect, Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QColor, QFont, QImage, QKeyEvent, QMouseEvent, QPainter, QPixmap
from PyQt6.QtWidgets import (
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QTabWidget,
    QVBoxLayout,
    QWidget,
    QPushButton
)
import sys
import time

import cv2
import depthai as dai
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from numpy.typing import NDArray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from gui.gui_node import GUINode
from gui.widgets.video_widget import (
    CameraDescription,
    CameraManager,
    CameraType,
    ClickableLabel,
    SwitchableVideoWidget,
    VideoWidget,
)
from rov_msgs.msg import Intrinsics
from rov_msgs.srv import CameraManage

FRAME_WIDTH = 816
FRAME_HEIGHT = 510
# FRAME_WIDTH = 1280
# FRAME_HEIGHT = 800

class Eye(IntEnum):
    LEFT = 0
    RIGHT = 1

class MeasurementTab(QWidget):

    def __init__(self) -> None:
        super().__init__()

        self.start_publisher = GUINode().create_publisher(Bool, 'measurement_pipeline', QoSPresetProfiles.DEFAULT.value)
        self.point_publisher = GUINode().create_publisher(Bool, 'retrieve_pointcloud', QoSPresetProfiles.DEFAULT.value)

        root_layout = QVBoxLayout()
        root_layout.addWidget(self.make_coarse_tab())

        start_btn = QPushButton('start pipeline')
        start_btn.clicked.connect(self.pub_start)
        end_btn = QPushButton('end pipeline')
        end_btn.clicked.connect(self.pub_end)
        point_btn = QPushButton('get point cloud')
        point_btn.clicked.connect(self.pub_point)

        root_layout.addWidget(start_btn)
        root_layout.addWidget(end_btn)
        root_layout.addWidget(point_btn)


        self.setLayout(root_layout)


    def make_coarse_tab(self) -> QWidget:

        cam_layout = QHBoxLayout()

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
                        CameraManager('manage_luxonis', CameraManage.Request.LUX_LEFT_RECT),
                    ),
                ),
                'switch_measure_stream',
            )
        }

        coarse_tab = QWidget()
        coarse_tab.setLayout(cam_layout)

        return coarse_tab
    
    def pub_start(self) -> None:
        print('\n\n\n\npublishing')
        self.start_publisher.publish(Bool(data=True))

    def pub_end(self) -> None:
        print('\n\n\n\npublishing')

        self.start_publisher.publish(Bool(data=False))

    def pub_point(self) -> None:
        print('\n\n\n\npublishing')

        self.point_publisher.publish(Bool(data=True))

    