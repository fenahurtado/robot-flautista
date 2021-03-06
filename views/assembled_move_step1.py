# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'assembled_move_step1.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(310, 253)
        self.gridLayout = QtWidgets.QGridLayout(Dialog)
        self.gridLayout.setObjectName("gridLayout")
        self.label = QtWidgets.QLabel(Dialog)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setWordWrap(True)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 2)
        self.label_2 = QtWidgets.QLabel(Dialog)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
        self.moveProfileComboBox = QtWidgets.QComboBox(Dialog)
        self.moveProfileComboBox.setObjectName("moveProfileComboBox")
        self.moveProfileComboBox.addItem("")
        self.moveProfileComboBox.addItem("")
        self.gridLayout.addWidget(self.moveProfileComboBox, 1, 1, 1, 1)
        self.label_3 = QtWidgets.QLabel(Dialog)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 2, 0, 1, 1)
        self.label_4 = QtWidgets.QLabel(Dialog)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 3, 0, 1, 1)
        self.segmentsSpinBox = QtWidgets.QSpinBox(Dialog)
        self.segmentsSpinBox.setMinimum(1)
        self.segmentsSpinBox.setMaximum(16)
        self.segmentsSpinBox.setObjectName("segmentsSpinBox")
        self.gridLayout.addWidget(self.segmentsSpinBox, 3, 1, 1, 1)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setCenterButtons(False)
        self.buttonBox.setObjectName("buttonBox")
        self.gridLayout.addWidget(self.buttonBox, 4, 0, 1, 2)
        self.dwellTimeSpinBox = QtWidgets.QSpinBox(Dialog)
        self.dwellTimeSpinBox.setEnabled(False)
        self.dwellTimeSpinBox.setObjectName("dwellTimeSpinBox")
        self.gridLayout.addWidget(self.dwellTimeSpinBox, 2, 1, 1, 1)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept) # type: ignore
        self.buttonBox.rejected.connect(Dialog.reject) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.label.setText(_translate("Dialog", "Programming Assembled Move - Step 1"))
        self.label_2.setText(_translate("Dialog", "Move profile:"))
        self.moveProfileComboBox.setItemText(0, _translate("Dialog", "Blend"))
        self.moveProfileComboBox.setItemText(1, _translate("Dialog", "Dwell"))
        self.label_3.setText(_translate("Dialog", "Dwell Time [ms]:"))
        self.label_4.setText(_translate("Dialog", "Number of segments:"))
