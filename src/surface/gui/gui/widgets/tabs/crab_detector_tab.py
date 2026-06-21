from dataclasses import dataclass
from enum import Enum, IntEnum
from typing import Generic, TypeGuard, TypeVar, override

from PyQt6.QtCore import Qt, pyqtSlot
from PyQt6.QtGui import QColor
from PyQt6.QtWidgets import (
    QHBoxLayout,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from gui.widgets.video_widget import (
    CameraDescription,
    CameraType,
    PauseableVideoWidget,
)
from rov_msgs.msg import Intrinsics

# CAM0_TOPIC = 'cam0/image_raw'
CAM1_TOPIC = 'cam1/image_raw'

FRAME_WIDTH = 816
FRAME_HEIGHT = 510
# FRAME_WIDTH = 1280
# FRAME_HEIGHT = 800

ZOOMED_WIDGET_SIZE = 405
ZOOM_DEFAULT_IDX = 2
ZOOMED_VIEWPORT_SIZES = (27, 45, 81, 135, 405)  # Odd factors of 405

LENGTH_SCALE_FACTOR = 1.34

PADDING = 200

POINTS_PER_EYE = 2

BASELINE_MM = 60.6
TUBE_RADIUS_MM = 40

DIVISION_SAFETY = 0.0001

BLACK = QColor(Qt.GlobalColor.black)

SHIPWRECK_BOW_LENGTH_CM = 30 + 16.6


class Eye(IntEnum):
    LEFT = 0
    RIGHT = 1


class Crosshair(Enum):
    Empty = 0
    Dot = 1


KEYS_TO_POINT_IDX = {
    Qt.Key.Key_1: (Eye.LEFT, 0),
    Qt.Key.Key_2: (Eye.LEFT, 1),
    Qt.Key.Key_3: (Eye.RIGHT, 0),
    Qt.Key.Key_4: (Eye.RIGHT, 1),
}


T = TypeVar('T', int, float)


@dataclass
class Point2D(Generic[T]):
    x: T
    y: T

    @override
    def __str__(self) -> str:
        return f'({round(self.x, 3)}, {round(self.y, 3)})'


@dataclass
class Point3D:
    x: float
    y: float
    z: float

    @override
    def __str__(self) -> str:
        return f'({round(self.x, 3)}, {round(self.y, 3)}, {round(self.z, 3)})'


def has_all_points(
    key_points: dict[Eye, list[Point2D[int] | None]],
) -> TypeGuard['dict[Eye, list[Point2D[int]]]']:
    return all(
        len(key_points[eye]) == POINTS_PER_EYE
        and all(point is not None for point in key_points[eye])
        for eye in Eye
    )


def format_length(length: float) -> str:
    return f'{length}, {length + SHIPWRECK_BOW_LENGTH_CM}'


class CrabDetectorTab(QWidget):
    # click_left_signal = pyqtSignal(QMouseEvent)
    # click_right_signal = pyqtSignal(QMouseEvent)

    def __init__(self) -> None:
        super().__init__()

        self.img_points: dict[Eye, list[Point2D[int] | None]] = {
            Eye.LEFT: [None, None],
            Eye.RIGHT: [None, None],
        }

        self.keys: dict[int, bool] = {
            Qt.Key.Key_1.value: False,
            Qt.Key.Key_2.value: False,
            Qt.Key.Key_3.value: False,
            Qt.Key.Key_4.value: False,
        }

        self.crosshair = Crosshair.Dot
        self.viewport_zoom_level = ZOOM_DEFAULT_IDX

        tabs = QTabWidget()
        tabs.addTab(self.make_coarse_tab(), 'Coarse')
        # tabs.addTab(self.make_fine_tab(), 'Fine')

        root_layout = QVBoxLayout()
        root_layout.addWidget(tabs)
        self.setLayout(root_layout)

        # Make sure we can get keyboard
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    def make_coarse_tab(self) -> QWidget:
        # self.click_left_signal.connect(self.click_left_slot)
        # self.click_right_signal.connect(self.click_right_slot)

        cam_layout = QHBoxLayout()

        # TODO: RESET THIS TO ACTUAL LUXONIS CAM ONCE IT'S WORKING AGAIN
        self.eye_widgets = {
            Eye.LEFT: PauseableVideoWidget(
                CameraDescription(
                    CameraType.ETHERNET,
                    CAM1_TOPIC,
                    'Down Camera',
                    FRAME_WIDTH,
                    FRAME_HEIGHT,
                ),
                #'switch_rect_stream',
                # make_label=lambda: ClickableLabel(self.click_left_signal),
            ),
        }

        for eye_widget in self.eye_widgets.values():
            cam_layout.addWidget(eye_widget)

        coarse_tab = QWidget()
        coarse_tab.setLayout(cam_layout)

        return coarse_tab

    @staticmethod
    def px_to_mm(px: float) -> float:
        # 3 um/px (https://docs.luxonis.com/hardware/sensors/OV9782)
        # / 1000 to get mm
        return px * 3 / 1000

    @pyqtSlot(Intrinsics)
    def intrinsics_left_slot(self, intrinsics: Intrinsics) -> None:
        self.intrinsics_left = intrinsics
        self.show_intrinsics()

    @pyqtSlot(Intrinsics)
    def intrinsics_right_slot(self, intrinsics: Intrinsics) -> None:
        self.intrinsics_right = intrinsics
        self.show_intrinsics()

    def show_intrinsics(self) -> None:
        if self.intrinsics_left is None or self.intrinsics_right is None:
            return

        focal_left_mm = Point2D(
            CrabDetectorTab.px_to_mm(self.intrinsics_left.fx),
            CrabDetectorTab.px_to_mm(self.intrinsics_left.fy),
        )
        focal_right_mm = Point2D(
            CrabDetectorTab.px_to_mm(self.intrinsics_right.fx),
            CrabDetectorTab.px_to_mm(self.intrinsics_right.fy),
        )
