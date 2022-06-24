from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox, QFileDialog, QLineEdit, QPushButton, QVBoxLayout, QGridLayout, QWidget, QLabel, QSpinBox, QMenu
)
from pyqtgraph.Qt import QtGui, QtCore, QtWidgets
from PyQt5.QtCore import QEventLoop, Qt

class ActionWidget(QWidget):
    def __init__(self, data, label, width=5, parent=None, context=None, index=0):
        super(ActionWidget, self).__init__(parent)
        self.data = data
        self.label = label
        self.width = width
        self.parent = parent
        self.context = context
        self.index = index
        self.performing = False
        self._generateUI()

    def _generateUI(self):
        main_layout = QGridLayout()
        #main_layout.SetFixedSize(130)
        self.setLayout(main_layout)
        self.setFixedWidth(int(100*self.width-6))
        #self.setFixedHeight(300)
        
        self.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        
        #

        #self.setFixedHeight(150)
        title = QLabel(self.label)
        title.setWordWrap(True)
        main_layout.addWidget(title, 0, 0, 1, 3)

        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.darkBlue)
        self.setPalette(p)

        # radius = 10.0
        # path = QtGui.QPainterPath()
        # path.addRoundedRect(QtCore.QRectF(self.rect()), radius, radius)
        # mask = QtGui.QRegion(path.toFillPolygon().toPolygon())
        # self.setMask(mask)
    
    def resizeEvent(self, event):
        radius = 10.0
        path = QtGui.QPainterPath()
        path.addRoundedRect(QtCore.QRectF(self.rect()), radius, radius)
        mask = QtGui.QRegion(path.toFillPolygon().toPolygon())
        self.setMask(mask)
        QtGui.QMainWindow.resizeEvent(self, event)

    def contextMenuEvent(self, event):
        if not self.performing:
            self.contextMenu = QMenu(self)
            #newAct = contextMenu.addAction("Open")
            self.editAct = self.contextMenu.addAction("Edit")
            
            if self.data['type'] != 0:
                self.addBefore = self.contextMenu.addAction("Add before")
                self.addAfter = self.contextMenu.addAction("Add after")
                self.quitAct = self.contextMenu.addAction("Delete")
            
            action = self.contextMenu.exec_(self.mapToGlobal(event.pos()))

            if self.data['type'] != 0:
                if action == self.quitAct:
                    #print('Index:', self.index)
                    self.deleteLater()
                    self.context.removeItem(self.context.itemAt(self.index))
                    if self.data['type'] == 2:
                        self.parent.finger_actions.pop(self.index)
                        self.parent.fingerActionsCount -= 1
                        self.parent.updateFingerIndexes()
                    else:
                        self.parent.phrase_actions.pop(self.index)
                        self.parent.actionsCount -= 1
                        self.parent.updateIndexes()
                    # while QApplication.hasPendingEvents():
                    #     QApplication.processEvents()
                    
                    
                elif action == self.addBefore:
                    if self.data['type'] != 2:
                        add = self.parent.add_action(pos=self.index)
                    else:
                        add = self.parent.add_fingers_action(pos=self.index)
                    if add:
                        # while QApplication.hasPendingEvents():
                        #     QApplication.processEvents()
                        self.parent.updateIndexes()
                elif action == self.addAfter:
                    if self.data['type'] != 2:
                        add = self.parent.add_action(pos=self.index+1)
                    else:
                        add = self.parent.add_fingers_action(pos=self.index+1)
                    if add:
                        # while QApplication.hasPendingEvents():
                        #     QApplication.processEvents()
                        self.parent.updateIndexes()
            if action == self.editAct:
                if self.data['type'] == 1:
                    edit = self.parent.add_action(pos=self.index, data=self.data)
                elif self.data['type'] == 2:
                    edit = self.parent.add_fingers_action(pos=self.index, data=self.data)
                else:
                    edit = self.parent.add_initial_position_action(data=self.data)
                #print(self.index, edit)
                if edit:
                    self.deleteLater()
                    self.context.removeItem(self.context.itemAt(self.index+1))
                    # while QApplication.hasPendingEvents():
                    #     QApplication.processEvents()
                    if self.data['type'] == 2:
                        self.parent.finger_actions.pop(self.index+1)
                        self.parent.fingerActionsCount -= 1
                        self.parent.updateFingerIndexes()
                    elif self.data['type'] == 1:
                        self.parent.phrase_actions.pop(self.index+1)
                        self.parent.actionsCount -= 1
                        self.parent.updateIndexes()
                    
                    #self.parent.scrollArea.horizontalScrollBar().setValue(self.parent.scrollArea.horizontalScrollBar().maximum())
                    
            
    
    def disable_context_menu(self):
        self.performing = True
    
    def enable_context_menu(self):
        self.performing = False

    def paint_green(self):
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.darkGreen)
        self.setPalette(p)
    
    def paint_blue(self):
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.darkBlue)
        self.setPalette(p)