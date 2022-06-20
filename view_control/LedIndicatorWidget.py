from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *


class LedWidget(QWidget):
    def __init__(self, label, parent=None, color='g'):
        super(LedWidget, self).__init__(parent)
        main_layout = QGridLayout()
        self.setLayout(main_layout)
        self.led = LedIndicator(self, color=color)
        self.led.setFixedWidth(30)
        main_layout.addWidget(self.led, 0, 0, 1, 1)

        self.title = QLabel(label)
        self.title.setWordWrap(True)
        main_layout.addWidget(self.title, 0, 1, 1, 1)
        # self.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)

class LedIndicator(QAbstractButton):
    scaledSize = 1000.0

    def __init__(self, parent=None, color='g'):
        QAbstractButton.__init__(self, parent)

        self.setMinimumSize(24, 24)
        self.setCheckable(True)

        # Green
        if color=='g':
            self.on_color_1 = QColor(0, 255, 0)
            self.on_color_2 = QColor(0, 192, 0)
            self.off_color_1 = QColor(0, 28, 0)
            self.off_color_2 = QColor(0, 128, 0)
        elif color=='r':
            self.on_color_1 = QColor(255, 0, 0)
            self.on_color_2 = QColor(176, 0, 0)
            self.off_color_1 = QColor(28, 0, 0)
            self.off_color_2 = QColor(156, 0, 0)
        elif color=='b':
            self.on_color_1 = QColor(0, 0, 255)
            self.on_color_2 = QColor(0, 0, 176)
            self.off_color_1 = QColor(0, 0, 28)
            self.off_color_2 = QColor(0, 0, 156)
        elif color=='y':
            self.on_color_1 = QColor(255, 255, 0)
            self.on_color_2 = QColor(176, 176, 0)
            self.off_color_1 = QColor(28, 28, 0)
            self.off_color_2 = QColor(156, 156, 0)

    def resizeEvent(self, QResizeEvent):
        self.update()

    def paintEvent(self, QPaintEvent):
        realSize = min(self.width(), self.height())

        painter = QPainter(self)
        pen = QPen(Qt.black)
        pen.setWidth(1)

        painter.setRenderHint(QPainter.Antialiasing)
        painter.translate(self.width() / 2, self.height() / 2)
        painter.scale(realSize / self.scaledSize, realSize / self.scaledSize)

        gradient = QRadialGradient(QPointF(-500, -500), 1500, QPointF(-500, -500))
        gradient.setColorAt(0, QColor(224, 224, 224))
        gradient.setColorAt(1, QColor(28, 28, 28))
        painter.setPen(pen)
        painter.setBrush(QBrush(gradient))
        painter.drawEllipse(QPointF(0, 0), 500, 500)

        gradient = QRadialGradient(QPointF(500, 500), 1500, QPointF(500, 500))
        gradient.setColorAt(0, QColor(224, 224, 224))
        gradient.setColorAt(1, QColor(28, 28, 28))
        painter.setPen(pen)
        painter.setBrush(QBrush(gradient))
        painter.drawEllipse(QPointF(0, 0), 450, 450)

        painter.setPen(pen)
        if self.isChecked():
            gradient = QRadialGradient(QPointF(-500, -500), 1500, QPointF(-500, -500))
            gradient.setColorAt(0, self.on_color_1)
            gradient.setColorAt(1, self.on_color_2)
        else:
            gradient = QRadialGradient(QPointF(500, 500), 1500, QPointF(500, 500))
            gradient.setColorAt(0, self.off_color_1)
            gradient.setColorAt(1, self.off_color_2)

        painter.setBrush(gradient)
        painter.drawEllipse(QPointF(0, 0), 400, 400)

    @pyqtProperty(QColor)
    def onColor1(self):
        return self.on_color_1

    @onColor1.setter
    def onColor1(self, color):
        self.on_color_1 = color

    @pyqtProperty(QColor)
    def onColor2(self):
        return self.on_color_2

    @onColor2.setter
    def onColor2(self, color):
        self.on_color_2 = color

    @pyqtProperty(QColor)
    def offColor1(self):
        return self.off_color_1

    @offColor1.setter
    def offColor1(self, color):
        self.off_color_1 = color

    @pyqtProperty(QColor)
    def offColor2(self):
        return self.off_color_2

    @offColor2.setter
    def offColor2(self, color):
        self.off_color_2 = color
