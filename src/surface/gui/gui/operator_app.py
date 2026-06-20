from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import QHBoxLayout, QPushButton, QTabWidget, QVBoxLayout, QWidget
from rclpy.qos import qos_profile_system_default

from gui.app import App
from gui.gui_node import GUINode
from gui.widgets.float_comm import FloatComm
from gui.widgets.flood_warning import FloodWarning
from gui.widgets.heartbeat import HeartbeatWidget
from gui.widgets.ip_widget import IPWidget
from gui.widgets.logger import Logger
from gui.widgets.tabs.general_debug_tab import GeneralDebugTab
from gui.widgets.tabs.shipwreck import ShipwreckTab
from gui.widgets.temperature import TemperatureSensor
from gui.widgets.timer import InteractiveTimer
from rov_msgs.msg import CropCam

SHIPWRECK_TEXT = 'Shipwreck'

TOPIC_CROP_CAM = 'cropCam'


class OperatorApp(App):
    changed_tabs = pyqtSignal(int)
    signal = pyqtSignal(CropCam)

    def __init__(self) -> None:
        super().__init__('operator_gui_node')
        self.crop = False
        self.signal.connect(self.refresh)

        GUINode().create_signal_subscription(CropCam, TOPIC_CROP_CAM, self.signal)
        self.publisher = GUINode().create_publisher(
            CropCam, TOPIC_CROP_CAM, qos_profile_system_default
        )

        self.setWindowTitle('Operator GUI - CWRUbotix ROV 2025')
        # Main tab
        main_tab = QWidget()
        main_layout = QHBoxLayout()
        main_tab.setLayout(main_layout)

        left_pane = QVBoxLayout()
        right_pane = QVBoxLayout()

        main_layout.addLayout(left_pane)
        main_layout.addLayout(right_pane)

        self.float_comm: FloatComm = FloatComm()
        left_pane.addWidget(self.float_comm)

        logger = Logger()
        left_pane.addWidget(logger)

        right_pane.addWidget(InteractiveTimer())
        right_pane.addWidget(HeartbeatWidget())
        right_pane.addWidget(FloodWarning())
        right_pane.addWidget(TemperatureSensor())
        right_pane.addWidget(IPWidget())
        right_pane.addStretch()

        # Add tabs to root
        root_layout = QVBoxLayout()
        self.setLayout(root_layout)

        self.tabs = QTabWidget()
        self.tabs.addTab(main_tab, 'Main')
        self.tabs.addTab(GeneralDebugTab(), 'General Debug')
        self.shipwreck_tab = ShipwreckTab()
        self.tabs.addTab(self.shipwreck_tab, SHIPWRECK_TEXT)
        self.crop_button = QPushButton()
        self.crop_button.setText('Crop Camera Output')
        root_layout.addWidget(self.crop_button)
        self.crop_button.clicked.connect(self.on_button_clicked)
        self.tabs.currentChanged.connect(self.changed_tabs)
        root_layout.addWidget(self.tabs)

        self.changed_tabs.connect(self.tab_change_slot)

    @pyqtSlot(int)
    def tab_change_slot(self, index: int) -> None:
        if self.tabs.tabText(index) == SHIPWRECK_TEXT:
            # Allow keyboard events
            self.shipwreck_tab.setFocus(Qt.FocusReason.TabFocusReason)

    # check is refresh runs
    @pyqtSlot(CropCam)
    def refresh(self, msg: CropCam) -> None:
        if msg.is_cropped:
            print('In refresh crop is true')
            self.crop = True
        else:
            print('In refresh crop is false')
            self.crop = False

    def on_button_clicked(self):
        print('**************************button clicked**********************')
        if self.crop:
            payload = CropCam(is_cropped=False)
            self.publisher.publish(payload)
            self.crop = False
            self.crop_button.setText('Crop Camera Output')
            print('crop should be false now')
        else:
            payload = CropCam(is_cropped=True)
            self.publisher.publish(payload)
            self.crop = True
            self.crop_button.setText('Enlarge Camera Output')
            print('crop should be true now')


def run_gui_operator() -> None:
    OperatorApp().run_gui()
