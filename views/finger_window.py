# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'finger_window.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(283, 170)
        self.gridLayout = QtWidgets.QGridLayout(Dialog)
        self.gridLayout.setObjectName("gridLayout")
        self.label_7 = QtWidgets.QLabel(Dialog)
        self.label_7.setObjectName("label_7")
        self.gridLayout.addWidget(self.label_7, 0, 0, 1, 1)
        self.noteComboBox = QtWidgets.QComboBox(Dialog)
        self.noteComboBox.setEnabled(True)
        self.noteComboBox.setObjectName("noteComboBox")
        self.noteComboBox.addItem("")
        self.noteComboBox.addItem("")
        self.noteComboBox.addItem("")
        self.noteComboBox.addItem("")
        self.noteComboBox.addItem("")
        self.noteComboBox.addItem("")
        self.noteComboBox.addItem("")
        self.noteComboBox.addItem("")
        self.noteComboBox.addItem("")
        self.noteComboBox.addItem("")
        self.noteComboBox.addItem("")
        self.noteComboBox.addItem("")
        self.gridLayout.addWidget(self.noteComboBox, 0, 1, 1, 1)
        self.label_5 = QtWidgets.QLabel(Dialog)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 1, 0, 1, 1)
        self.durationSpinBox = QtWidgets.QDoubleSpinBox(Dialog)
        self.durationSpinBox.setMinimum(0.1)
        self.durationSpinBox.setProperty("value", 1.0)
        self.durationSpinBox.setObjectName("durationSpinBox")
        self.gridLayout.addWidget(self.durationSpinBox, 1, 1, 1, 1)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.gridLayout.addWidget(self.buttonBox, 2, 0, 1, 2)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept) # type: ignore
        self.buttonBox.rejected.connect(Dialog.reject) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.label_7.setText(_translate("Dialog", "Note"))
        self.noteComboBox.setItemText(0, _translate("Dialog", "A"))
        self.noteComboBox.setItemText(1, _translate("Dialog", "A#"))
        self.noteComboBox.setItemText(2, _translate("Dialog", "B"))
        self.noteComboBox.setItemText(3, _translate("Dialog", "C"))
        self.noteComboBox.setItemText(4, _translate("Dialog", "C#"))
        self.noteComboBox.setItemText(5, _translate("Dialog", "D"))
        self.noteComboBox.setItemText(6, _translate("Dialog", "D#"))
        self.noteComboBox.setItemText(7, _translate("Dialog", "E"))
        self.noteComboBox.setItemText(8, _translate("Dialog", "F"))
        self.noteComboBox.setItemText(9, _translate("Dialog", "F#"))
        self.noteComboBox.setItemText(10, _translate("Dialog", "G"))
        self.noteComboBox.setItemText(11, _translate("Dialog", "G#"))
        self.label_5.setText(_translate("Dialog", "Duration"))
