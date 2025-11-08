from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QHBoxLayout, QLabel, QVBoxLayout, QWidget

class MillisecondTimerWidget(QLabel):
    def __init__(self) -> None:
        super().__init__()

        timer_layout = QVBoxLayout()

        font = QFont('Arial', 14)

        self.timer_label = QLabel('00:00:000')
        self.timer_label.setFont(font)
        timer_layout.addWidget(self.timer_label)

        self.setLayout(timer_layout)

    def update_time(self, milliseconds: int) -> None:
        minutes = milliseconds // 60000
        seconds = (milliseconds % 60000) // 1000
        millis = milliseconds % 1000
        self.setText(f'{minutes:02}:{seconds:02}:{millis:03}')