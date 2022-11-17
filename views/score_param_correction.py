# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'score_param_correction.ui'
#
# Created by: PyQt5 UI code generator 5.15.7
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(312, 411)
        self.gridLayout = QtWidgets.QGridLayout(Dialog)
        self.gridLayout.setObjectName("gridLayout")
        self.label = QtWidgets.QLabel(Dialog)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.RSpinBox = QtWidgets.QDoubleSpinBox(Dialog)
        self.RSpinBox.setMinimum(-10.0)
        self.RSpinBox.setMaximum(10.0)
        self.RSpinBox.setSingleStep(0.5)
        self.RSpinBox.setObjectName("RSpinBox")
        self.gridLayout.addWidget(self.RSpinBox, 0, 2, 1, 1)
        self.label_2 = QtWidgets.QLabel(Dialog)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 2)
        self.ThetaSpinBox = QtWidgets.QDoubleSpinBox(Dialog)
        self.ThetaSpinBox.setMinimum(-10.0)
        self.ThetaSpinBox.setMaximum(10.0)
        self.ThetaSpinBox.setSingleStep(0.5)
        self.ThetaSpinBox.setObjectName("ThetaSpinBox")
        self.gridLayout.addWidget(self.ThetaSpinBox, 1, 2, 1, 1)
        self.label_3 = QtWidgets.QLabel(Dialog)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 2, 0, 1, 2)
        self.OffsetSpinBox = QtWidgets.QDoubleSpinBox(Dialog)
        self.OffsetSpinBox.setMinimum(-10.0)
        self.OffsetSpinBox.setMaximum(10.0)
        self.OffsetSpinBox.setSingleStep(0.5)
        self.OffsetSpinBox.setObjectName("OffsetSpinBox")
        self.gridLayout.addWidget(self.OffsetSpinBox, 2, 2, 1, 1)
        self.label_4 = QtWidgets.QLabel(Dialog)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 3, 0, 1, 2)
        self.FlowSpinBox = QtWidgets.QDoubleSpinBox(Dialog)
        self.FlowSpinBox.setMinimum(-50.0)
        self.FlowSpinBox.setMaximum(50.0)
        self.FlowSpinBox.setSingleStep(0.5)
        self.FlowSpinBox.setProperty("value", 0.0)
        self.FlowSpinBox.setObjectName("FlowSpinBox")
        self.gridLayout.addWidget(self.FlowSpinBox, 3, 2, 1, 1)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.gridLayout.addWidget(self.buttonBox, 4, 1, 1, 2)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept) # type: ignore
        self.buttonBox.rejected.connect(Dialog.reject) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.label.setText(_translate("Dialog", "R correction"))
        self.label_2.setText(_translate("Dialog", "Theta correction"))
        self.label_3.setText(_translate("Dialog", "Offset correction"))
        self.label_4.setText(_translate("Dialog", "Flow correction"))
