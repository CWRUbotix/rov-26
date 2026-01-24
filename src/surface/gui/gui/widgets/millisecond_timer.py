from PyQt6.QtCore import Qt, QTimer, pyqtSlot
from PyQt6.QtGui import QFont
from PyQt6.QtWidgets import QLabel, QVBoxLayout, QWidget

class MillisecondTimerWidget(QWidget):
    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()
        font = QFont('Arial', 14)

        self.timer_label = QLabel('00:00:000')
        self.timer_label.setFont(font)
        self.timer_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(self.timer_label)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

        self._elapsed_ms: int = 0
        self._timer = QTimer(self)
        self._timer.setInterval(10)
        self._timer.timeout.connect(self._on_timeout)
        self._timer.start()        

    def _on_timeout(self) -> None:
        self._elapsed_ms += self._timer.interval()
        self.update_time(self._elapsed_ms)

    def update_time(self, milliseconds: int) -> None:
        minutes = milliseconds // 60000
        seconds = (milliseconds % 60000) // 1000
        millis = milliseconds % 1000
        self.timer_label.setText(f'{minutes:02}:{seconds:02}:{millis:03}')
