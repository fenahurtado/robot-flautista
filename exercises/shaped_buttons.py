from PyQt5 import QtGui, QtCore, QtWidgets

class ArrowButton(QtWidgets.QPushButton):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(30, 30)
        self.color = QtGui.QColor(100, 100, 100, 255)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtCore.Qt.NoPen)
        if self.isDown():
            color = self.color.darker(150)
        else:
            color = self.color
        painter.setBrush(color)
        arrow_head = QtGui.QPolygonF([
            QtCore.QPointF(self.width() - 20, self.height() / 2 - 7),
            QtCore.QPointF(self.width() - 30, self.height() / 2),
            QtCore.QPointF(self.width() - 20, self.height() / 2 + 7),
        ])
        painter.drawPolygon(arrow_head)
        tail_width = 7
        tail_height = 14
        tail_path = QtGui.QPainterPath()
        tail_path.moveTo(self.width() - 20, self.height() / 2 - 7)
        tail_path.lineTo(self.width() - 20 - tail_width, self.height() / 2 - 7)
        tail_path.lineTo(self.width() - 20 - tail_width, self.height() / 2 - 7 - tail_height)
        tail_path.lineTo(self.width() - 20 - tail_width * 2, self.height() / 2)
        tail_path.lineTo(self.width() - 20 - tail_width, self.height() / 2 + 7 + tail_height)
        tail_path.lineTo(self.width() - 20 - tail_width, self.height() / 2 + 7)
        tail_path.lineTo(self.width() - 20, self.height() / 2 + 7)
        painter.drawPath(tail_path)

    def mousePressEvent(self, event):
        self.update()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self.update()
        super().mouseReleaseEvent(event)


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName('ArrowButton')

    main = QtWidgets.QWidget()
    layout = QtWidgets.QHBoxLayout()
    main.setLayout(layout)

    arrow_button = ArrowButton()
    layout.addWidget(arrow_button)

    main.show()
    sys.exit(app.exec_())
