# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'move_edit_window.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(369, 749)
        self.gridLayout_2 = QtWidgets.QGridLayout(Dialog)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.moveParametersGroupBox = QtWidgets.QGroupBox(Dialog)
        self.moveParametersGroupBox.setObjectName("moveParametersGroupBox")
        self.gridLayout = QtWidgets.QGridLayout(self.moveParametersGroupBox)
        self.gridLayout.setObjectName("gridLayout")
        self.deformationSpinBox = QtWidgets.QDoubleSpinBox(self.moveParametersGroupBox)
        self.deformationSpinBox.setWhatsThis("")
        self.deformationSpinBox.setObjectName("deformationSpinBox")
        self.gridLayout.addWidget(self.deformationSpinBox, 9, 1, 1, 1)
        self.label_11 = QtWidgets.QLabel(self.moveParametersGroupBox)
        self.label_11.setObjectName("label_11")
        self.gridLayout.addWidget(self.label_11, 4, 0, 1, 1)
        self.vibratoAmplitudeSpinBox = QtWidgets.QDoubleSpinBox(self.moveParametersGroupBox)
        self.vibratoAmplitudeSpinBox.setWhatsThis("")
        self.vibratoAmplitudeSpinBox.setObjectName("vibratoAmplitudeSpinBox")
        self.gridLayout.addWidget(self.vibratoAmplitudeSpinBox, 10, 1, 1, 1)
        self.plotFlowRampButton = QtWidgets.QPushButton(self.moveParametersGroupBox)
        self.plotFlowRampButton.setObjectName("plotFlowRampButton")
        self.gridLayout.addWidget(self.plotFlowRampButton, 12, 0, 1, 2)
        self.label_6 = QtWidgets.QLabel(self.moveParametersGroupBox)
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 9, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.moveParametersGroupBox)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)
        self.thetaSpinBox = QtWidgets.QDoubleSpinBox(self.moveParametersGroupBox)
        self.thetaSpinBox.setMinimum(-180.0)
        self.thetaSpinBox.setMaximum(180.0)
        self.thetaSpinBox.setObjectName("thetaSpinBox")
        self.gridLayout.addWidget(self.thetaSpinBox, 1, 1, 1, 1)
        self.vibratoFrequencySpinBox = QtWidgets.QDoubleSpinBox(self.moveParametersGroupBox)
        self.vibratoFrequencySpinBox.setMinimum(0.0)
        self.vibratoFrequencySpinBox.setProperty("value", 0.0)
        self.vibratoFrequencySpinBox.setObjectName("vibratoFrequencySpinBox")
        self.gridLayout.addWidget(self.vibratoFrequencySpinBox, 11, 1, 1, 1)
        self.label_8 = QtWidgets.QLabel(self.moveParametersGroupBox)
        self.label_8.setObjectName("label_8")
        self.gridLayout.addWidget(self.label_8, 11, 0, 1, 1)
        self.accelerationValueSpinBox = QtWidgets.QDoubleSpinBox(self.moveParametersGroupBox)
        self.accelerationValueSpinBox.setObjectName("accelerationValueSpinBox")
        self.gridLayout.addWidget(self.accelerationValueSpinBox, 4, 1, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.moveParametersGroupBox)
        self.label_9.setObjectName("label_9")
        self.gridLayout.addWidget(self.label_9, 10, 0, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.moveParametersGroupBox)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 8, 0, 1, 1)
        self.radiusSpinBox = QtWidgets.QDoubleSpinBox(self.moveParametersGroupBox)
        self.radiusSpinBox.setMaximum(150.0)
        self.radiusSpinBox.setObjectName("radiusSpinBox")
        self.gridLayout.addWidget(self.radiusSpinBox, 0, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.moveParametersGroupBox)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 1, 0, 1, 1)
        self.label_3 = QtWidgets.QLabel(self.moveParametersGroupBox)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 2, 0, 1, 1)
        self.offsetSpinBox = QtWidgets.QDoubleSpinBox(self.moveParametersGroupBox)
        self.offsetSpinBox.setMinimum(-100.0)
        self.offsetSpinBox.setMaximum(100.0)
        self.offsetSpinBox.setObjectName("offsetSpinBox")
        self.gridLayout.addWidget(self.offsetSpinBox, 2, 1, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.moveParametersGroupBox)
        self.label_10.setObjectName("label_10")
        self.gridLayout.addWidget(self.label_10, 6, 0, 1, 1)
        self.plotMovementButton = QtWidgets.QPushButton(self.moveParametersGroupBox)
        self.plotMovementButton.setObjectName("plotMovementButton")
        self.gridLayout.addWidget(self.plotMovementButton, 7, 0, 1, 2)
        self.jerkSpinBox = QtWidgets.QDoubleSpinBox(self.moveParametersGroupBox)
        self.jerkSpinBox.setObjectName("jerkSpinBox")
        self.gridLayout.addWidget(self.jerkSpinBox, 6, 1, 1, 1)
        self.label_12 = QtWidgets.QLabel(self.moveParametersGroupBox)
        self.label_12.setObjectName("label_12")
        self.gridLayout.addWidget(self.label_12, 5, 0, 1, 1)
        self.decelerationValueSpinBox = QtWidgets.QDoubleSpinBox(self.moveParametersGroupBox)
        self.decelerationValueSpinBox.setObjectName("decelerationValueSpinBox")
        self.gridLayout.addWidget(self.decelerationValueSpinBox, 5, 1, 1, 1)
        self.flowSpinBox = QtWidgets.QDoubleSpinBox(self.moveParametersGroupBox)
        self.flowSpinBox.setMaximum(50.0)
        self.flowSpinBox.setObjectName("flowSpinBox")
        self.gridLayout.addWidget(self.flowSpinBox, 8, 1, 1, 1)
        self.gridLayout_2.addWidget(self.moveParametersGroupBox, 2, 0, 1, 2)
        self.durationSpinBox = QtWidgets.QDoubleSpinBox(Dialog)
        self.durationSpinBox.setMinimum(0.1)
        self.durationSpinBox.setProperty("value", 1.0)
        self.durationSpinBox.setObjectName("durationSpinBox")
        self.gridLayout_2.addWidget(self.durationSpinBox, 1, 1, 1, 1)
        self.label_7 = QtWidgets.QLabel(Dialog)
        self.label_7.setObjectName("label_7")
        self.gridLayout_2.addWidget(self.label_7, 0, 0, 1, 1)
        self.typeComboBox = QtWidgets.QComboBox(Dialog)
        self.typeComboBox.setEnabled(True)
        self.typeComboBox.setObjectName("typeComboBox")
        self.typeComboBox.addItem("")
        self.typeComboBox.addItem("")
        self.gridLayout_2.addWidget(self.typeComboBox, 0, 1, 1, 1)
        self.label_5 = QtWidgets.QLabel(Dialog)
        self.label_5.setObjectName("label_5")
        self.gridLayout_2.addWidget(self.label_5, 1, 0, 1, 1)
        self.buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.gridLayout_2.addWidget(self.buttonBox, 4, 1, 1, 1)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout_2.addItem(spacerItem, 3, 1, 1, 1)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept) # type: ignore
        self.buttonBox.rejected.connect(Dialog.reject) # type: ignore
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Dialog"))
        self.moveParametersGroupBox.setTitle(_translate("Dialog", "Move Parameters"))
        self.label_11.setText(_translate("Dialog", "Acceleration Value"))
        self.plotFlowRampButton.setText(_translate("Dialog", "Plot Flow Ramp"))
        self.label_6.setText(_translate("Dialog", "Flow Ramp deformation"))
        self.label.setText(_translate("Dialog", "Final Radius"))
        self.label_8.setText(_translate("Dialog", "Vibrato Frequency"))
        self.label_9.setText(_translate("Dialog", "Vibrato Amplitude"))
        self.label_4.setText(_translate("Dialog", "Final Flow Rate"))
        self.label_2.setText(_translate("Dialog", "Final Incidence Angle"))
        self.label_3.setText(_translate("Dialog", "Final Jet Offset"))
        self.label_10.setText(_translate("Dialog", "Acceleration Jerk"))
        self.plotMovementButton.setText(_translate("Dialog", "Plot Movement"))
        self.label_12.setText(_translate("Dialog", "Deceleration Value"))
        self.label_7.setText(_translate("Dialog", "Action Type"))
        self.typeComboBox.setItemText(0, _translate("Dialog", "Stay"))
        self.typeComboBox.setItemText(1, _translate("Dialog", "Move"))
        self.label_5.setText(_translate("Dialog", "Duration"))
