# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'amci_driver.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1018, 768)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.gridLayout_11 = QtWidgets.QGridLayout(self.centralwidget)
        self.gridLayout_11.setObjectName("gridLayout_11")
        self.groupBox_6 = QtWidgets.QGroupBox(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_6.sizePolicy().hasHeightForWidth())
        self.groupBox_6.setSizePolicy(sizePolicy)
        self.groupBox_6.setMinimumSize(QtCore.QSize(0, 80))
        self.groupBox_6.setObjectName("groupBox_6")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.groupBox_6)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.hybridCheckBox = QtWidgets.QCheckBox(self.groupBox_6)
        self.hybridCheckBox.setEnabled(False)
        self.hybridCheckBox.setObjectName("hybridCheckBox")
        self.horizontalLayout_5.addWidget(self.hybridCheckBox)
        self.blendDirCheckBox = QtWidgets.QCheckBox(self.groupBox_6)
        self.blendDirCheckBox.setObjectName("blendDirCheckBox")
        self.horizontalLayout_5.addWidget(self.blendDirCheckBox)
        self.encoderMoveCheckBox = QtWidgets.QCheckBox(self.groupBox_6)
        self.encoderMoveCheckBox.setEnabled(False)
        self.encoderMoveCheckBox.setObjectName("encoderMoveCheckBox")
        self.horizontalLayout_5.addWidget(self.encoderMoveCheckBox)
        self.electricGearCheckBox = QtWidgets.QCheckBox(self.groupBox_6)
        self.electricGearCheckBox.setEnabled(False)
        self.electricGearCheckBox.setObjectName("electricGearCheckBox")
        self.horizontalLayout_5.addWidget(self.electricGearCheckBox)
        self.indexedMoveCheckBox = QtWidgets.QCheckBox(self.groupBox_6)
        self.indexedMoveCheckBox.setObjectName("indexedMoveCheckBox")
        self.horizontalLayout_5.addWidget(self.indexedMoveCheckBox)
        self.dwellMoveCheckBox = QtWidgets.QCheckBox(self.groupBox_6)
        self.dwellMoveCheckBox.setObjectName("dwellMoveCheckBox")
        self.horizontalLayout_5.addWidget(self.dwellMoveCheckBox)
        self.outputStateCheckBox = QtWidgets.QCheckBox(self.groupBox_6)
        self.outputStateCheckBox.setObjectName("outputStateCheckBox")
        self.horizontalLayout_5.addWidget(self.outputStateCheckBox)
        self.clearHWFaultCheckBox = QtWidgets.QCheckBox(self.groupBox_6)
        self.clearHWFaultCheckBox.setEnabled(False)
        self.clearHWFaultCheckBox.setObjectName("clearHWFaultCheckBox")
        self.horizontalLayout_5.addWidget(self.clearHWFaultCheckBox)
        self.gridLayout_4.addLayout(self.horizontalLayout_5, 0, 0, 1, 1)
        self.gridLayout_11.addWidget(self.groupBox_6, 2, 0, 1, 3)
        self.stateGroupBox = QtWidgets.QGroupBox(self.centralwidget)
        self.stateGroupBox.setObjectName("stateGroupBox")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.stateGroupBox)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.commandStatus0Layout = QtWidgets.QGridLayout()
        self.commandStatus0Layout.setObjectName("commandStatus0Layout")
        self.gridLayout_2.addLayout(self.commandStatus0Layout, 1, 0, 1, 1)
        self.gridLayout_11.addWidget(self.stateGroupBox, 6, 0, 1, 3)
        self.groupBox_3 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_3.setObjectName("groupBox_3")
        self.gridLayout_6 = QtWidgets.QGridLayout(self.groupBox_3)
        self.gridLayout_6.setObjectName("gridLayout_6")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.presetEncoderButton = QtWidgets.QPushButton(self.groupBox_3)
        self.presetEncoderButton.setObjectName("presetEncoderButton")
        self.horizontalLayout.addWidget(self.presetEncoderButton)
        self.runAssemblyButton = QtWidgets.QPushButton(self.groupBox_3)
        self.runAssemblyButton.setObjectName("runAssemblyButton")
        self.horizontalLayout.addWidget(self.runAssemblyButton)
        self.prgAssemblyButton = QtWidgets.QPushButton(self.groupBox_3)
        self.prgAssemblyButton.setObjectName("prgAssemblyButton")
        self.horizontalLayout.addWidget(self.prgAssemblyButton)
        self.resetErrorsButton = QtWidgets.QPushButton(self.groupBox_3)
        self.resetErrorsButton.setObjectName("resetErrorsButton")
        self.horizontalLayout.addWidget(self.resetErrorsButton)
        self.presetPositionButton = QtWidgets.QPushButton(self.groupBox_3)
        self.presetPositionButton.setObjectName("presetPositionButton")
        self.horizontalLayout.addWidget(self.presetPositionButton)
        self.manualCCWButton = QtWidgets.QPushButton(self.groupBox_3)
        self.manualCCWButton.setObjectName("manualCCWButton")
        self.horizontalLayout.addWidget(self.manualCCWButton)
        self.manualCWButton = QtWidgets.QPushButton(self.groupBox_3)
        self.manualCWButton.setObjectName("manualCWButton")
        self.horizontalLayout.addWidget(self.manualCWButton)
        self.gridLayout_6.addLayout(self.horizontalLayout, 0, 0, 1, 1)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.homeCCWButton = QtWidgets.QPushButton(self.groupBox_3)
        self.homeCCWButton.setObjectName("homeCCWButton")
        self.horizontalLayout_2.addWidget(self.homeCCWButton)
        self.homeCWButton = QtWidgets.QPushButton(self.groupBox_3)
        self.homeCWButton.setObjectName("homeCWButton")
        self.horizontalLayout_2.addWidget(self.homeCWButton)
        self.immediateStopButton = QtWidgets.QPushButton(self.groupBox_3)
        self.immediateStopButton.setObjectName("immediateStopButton")
        self.horizontalLayout_2.addWidget(self.immediateStopButton)
        self.resumeMoveButton = QtWidgets.QPushButton(self.groupBox_3)
        self.resumeMoveButton.setObjectName("resumeMoveButton")
        self.horizontalLayout_2.addWidget(self.resumeMoveButton)
        self.holdMoveButton = QtWidgets.QPushButton(self.groupBox_3)
        self.holdMoveButton.setObjectName("holdMoveButton")
        self.horizontalLayout_2.addWidget(self.holdMoveButton)
        self.relativeMoveButton = QtWidgets.QPushButton(self.groupBox_3)
        self.relativeMoveButton.setObjectName("relativeMoveButton")
        self.horizontalLayout_2.addWidget(self.relativeMoveButton)
        self.absoluteMoveButton = QtWidgets.QPushButton(self.groupBox_3)
        self.absoluteMoveButton.setEnabled(True)
        self.absoluteMoveButton.setObjectName("absoluteMoveButton")
        self.horizontalLayout_2.addWidget(self.absoluteMoveButton)
        self.gridLayout_6.addLayout(self.horizontalLayout_2, 1, 0, 1, 1)
        self.gridLayout_11.addWidget(self.groupBox_3, 1, 0, 1, 3)
        self.groupBox_7 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_7.setObjectName("groupBox_7")
        self.gridLayout_8 = QtWidgets.QGridLayout(self.groupBox_7)
        self.gridLayout_8.setObjectName("gridLayout_8")
        self.gridLayout_7 = QtWidgets.QGridLayout()
        self.gridLayout_7.setObjectName("gridLayout_7")
        self.label_3 = QtWidgets.QLabel(self.groupBox_7)
        self.label_3.setObjectName("label_3")
        self.gridLayout_7.addWidget(self.label_3, 2, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.groupBox_7)
        self.label.setObjectName("label")
        self.gridLayout_7.addWidget(self.label, 0, 0, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.groupBox_7)
        self.label_2.setObjectName("label_2")
        self.gridLayout_7.addWidget(self.label_2, 1, 0, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.groupBox_7)
        self.label_4.setObjectName("label_4")
        self.gridLayout_7.addWidget(self.label_4, 3, 0, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.groupBox_7)
        self.label_9.setObjectName("label_9")
        self.gridLayout_7.addWidget(self.label_9, 4, 0, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.groupBox_7)
        self.label_10.setObjectName("label_10")
        self.gridLayout_7.addWidget(self.label_10, 5, 0, 1, 1)
        self.jerkSpinBox = QtWidgets.QSpinBox(self.groupBox_7)
        self.jerkSpinBox.setMaximum(5000)
        self.jerkSpinBox.setObjectName("jerkSpinBox")
        self.gridLayout_7.addWidget(self.jerkSpinBox, 5, 1, 1, 1)
        self.dwellDelaySpinBox = QtWidgets.QSpinBox(self.groupBox_7)
        self.dwellDelaySpinBox.setMaximum(65535)
        self.dwellDelaySpinBox.setObjectName("dwellDelaySpinBox")
        self.gridLayout_7.addWidget(self.dwellDelaySpinBox, 4, 1, 1, 1)
        self.decelerationSpinBox = QtWidgets.QSpinBox(self.groupBox_7)
        self.decelerationSpinBox.setMinimum(1)
        self.decelerationSpinBox.setMaximum(5000)
        self.decelerationSpinBox.setProperty("value", 100)
        self.decelerationSpinBox.setObjectName("decelerationSpinBox")
        self.gridLayout_7.addWidget(self.decelerationSpinBox, 3, 1, 1, 1)
        self.accelerationSpinBox = QtWidgets.QSpinBox(self.groupBox_7)
        self.accelerationSpinBox.setMinimum(1)
        self.accelerationSpinBox.setMaximum(5000)
        self.accelerationSpinBox.setProperty("value", 100)
        self.accelerationSpinBox.setObjectName("accelerationSpinBox")
        self.gridLayout_7.addWidget(self.accelerationSpinBox, 2, 1, 1, 1)
        self.speedSpinBox = QtWidgets.QSpinBox(self.groupBox_7)
        self.speedSpinBox.setMinimum(1)
        self.speedSpinBox.setMaximum(2999999)
        self.speedSpinBox.setProperty("value", 200)
        self.speedSpinBox.setObjectName("speedSpinBox")
        self.gridLayout_7.addWidget(self.speedSpinBox, 1, 1, 1, 1)
        self.positionSpinBox = QtWidgets.QSpinBox(self.groupBox_7)
        self.positionSpinBox.setMinimum(-8388607)
        self.positionSpinBox.setMaximum(8388607)
        self.positionSpinBox.setObjectName("positionSpinBox")
        self.gridLayout_7.addWidget(self.positionSpinBox, 0, 1, 1, 1)
        self.gridLayout_8.addLayout(self.gridLayout_7, 0, 0, 1, 1)
        self.gridLayout_11.addWidget(self.groupBox_7, 3, 0, 2, 1)
        self.groupBox_8 = QtWidgets.QGroupBox(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_8.sizePolicy().hasHeightForWidth())
        self.groupBox_8.setSizePolicy(sizePolicy)
        self.groupBox_8.setMinimumSize(QtCore.QSize(250, 0))
        self.groupBox_8.setObjectName("groupBox_8")
        self.gridLayout_10 = QtWidgets.QGridLayout(self.groupBox_8)
        self.gridLayout_10.setObjectName("gridLayout_10")
        self.gridLayout_9 = QtWidgets.QGridLayout()
        self.gridLayout_9.setObjectName("gridLayout_9")
        self.label_11 = QtWidgets.QLabel(self.groupBox_8)
        self.label_11.setObjectName("label_11")
        self.gridLayout_9.addWidget(self.label_11, 0, 0, 1, 1)
        self.label_13 = QtWidgets.QLabel(self.groupBox_8)
        self.label_13.setObjectName("label_13")
        self.gridLayout_9.addWidget(self.label_13, 1, 0, 1, 1)
        self.capturedEncoderPositionLabel = QtWidgets.QLabel(self.groupBox_8)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.capturedEncoderPositionLabel.setFont(font)
        self.capturedEncoderPositionLabel.setObjectName("capturedEncoderPositionLabel")
        self.gridLayout_9.addWidget(self.capturedEncoderPositionLabel, 2, 1, 1, 1)
        self.currentPositionLabel = QtWidgets.QLabel(self.groupBox_8)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.currentPositionLabel.sizePolicy().hasHeightForWidth())
        self.currentPositionLabel.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.currentPositionLabel.setFont(font)
        self.currentPositionLabel.setObjectName("currentPositionLabel")
        self.gridLayout_9.addWidget(self.currentPositionLabel, 0, 1, 1, 1)
        self.encoderPositionLabel = QtWidgets.QLabel(self.groupBox_8)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.encoderPositionLabel.setFont(font)
        self.encoderPositionLabel.setObjectName("encoderPositionLabel")
        self.gridLayout_9.addWidget(self.encoderPositionLabel, 1, 1, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.groupBox_8)
        self.label_5.setObjectName("label_5")
        self.gridLayout_9.addWidget(self.label_5, 3, 0, 1, 1)
        self.label_15 = QtWidgets.QLabel(self.groupBox_8)
        self.label_15.setObjectName("label_15")
        self.gridLayout_9.addWidget(self.label_15, 2, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.groupBox_8)
        self.label_6.setObjectName("label_6")
        self.gridLayout_9.addWidget(self.label_6, 4, 0, 1, 1)
        self.motorCurrentLabel = QtWidgets.QLabel(self.groupBox_8)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.motorCurrentLabel.setFont(font)
        self.motorCurrentLabel.setObjectName("motorCurrentLabel")
        self.gridLayout_9.addWidget(self.motorCurrentLabel, 3, 1, 1, 1)
        self.accelerationJerkLabel = QtWidgets.QLabel(self.groupBox_8)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setWeight(75)
        self.accelerationJerkLabel.setFont(font)
        self.accelerationJerkLabel.setObjectName("accelerationJerkLabel")
        self.gridLayout_9.addWidget(self.accelerationJerkLabel, 4, 1, 1, 1)
        self.gridLayout_10.addLayout(self.gridLayout_9, 0, 0, 1, 1)
        self.gridLayout_11.addWidget(self.groupBox_8, 3, 1, 1, 1)
        self.groupBox_4 = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_4.setObjectName("groupBox_4")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.groupBox_4)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.commandStatus1Layout = QtWidgets.QGridLayout()
        self.commandStatus1Layout.setObjectName("commandStatus1Layout")
        self.gridLayout_3.addLayout(self.commandStatus1Layout, 0, 0, 1, 1)
        self.gridLayout_11.addWidget(self.groupBox_4, 3, 2, 2, 1)
        self.disableDriverButton = QtWidgets.QPushButton(self.centralwidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.disableDriverButton.sizePolicy().hasHeightForWidth())
        self.disableDriverButton.setSizePolicy(sizePolicy)
        self.disableDriverButton.setObjectName("disableDriverButton")
        self.gridLayout_11.addWidget(self.disableDriverButton, 4, 1, 1, 1)
        self.configureButton = QtWidgets.QPushButton(self.centralwidget)
        self.configureButton.setObjectName("configureButton")
        self.gridLayout_11.addWidget(self.configureButton, 0, 0, 1, 3)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1018, 29))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actionConfiguration = QtWidgets.QAction(MainWindow)
        self.actionConfiguration.setObjectName("actionConfiguration")

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.groupBox_6.setTitle(_translate("MainWindow", "Command Word 1"))
        self.hybridCheckBox.setText(_translate("MainWindow", "Hybrid"))
        self.blendDirCheckBox.setText(_translate("MainWindow", "Blend Dir"))
        self.encoderMoveCheckBox.setText(_translate("MainWindow", "Encoder Move"))
        self.electricGearCheckBox.setText(_translate("MainWindow", "Electr. Gear"))
        self.indexedMoveCheckBox.setText(_translate("MainWindow", "Indexed Move"))
        self.dwellMoveCheckBox.setText(_translate("MainWindow", "Dwell Move"))
        self.outputStateCheckBox.setText(_translate("MainWindow", "Output State"))
        self.clearHWFaultCheckBox.setText(_translate("MainWindow", "Clear HW Fault"))
        self.stateGroupBox.setTitle(_translate("MainWindow", "Status Word 1"))
        self.groupBox_3.setTitle(_translate("MainWindow", "Command Word 0"))
        self.presetEncoderButton.setText(_translate("MainWindow", "Preset Encoder"))
        self.runAssemblyButton.setText(_translate("MainWindow", "Run Assem."))
        self.prgAssemblyButton.setText(_translate("MainWindow", "PRG Assem."))
        self.resetErrorsButton.setText(_translate("MainWindow", "Reset Errors"))
        self.presetPositionButton.setText(_translate("MainWindow", "Preset Position"))
        self.manualCCWButton.setText(_translate("MainWindow", "Manual CCW"))
        self.manualCWButton.setText(_translate("MainWindow", "Manual CW"))
        self.homeCCWButton.setText(_translate("MainWindow", "Home CCW"))
        self.homeCWButton.setText(_translate("MainWindow", "Home CW"))
        self.immediateStopButton.setText(_translate("MainWindow", "Immed Stop"))
        self.resumeMoveButton.setText(_translate("MainWindow", "Resume Move"))
        self.holdMoveButton.setText(_translate("MainWindow", "Hold Move"))
        self.relativeMoveButton.setText(_translate("MainWindow", "Relative Move"))
        self.absoluteMoveButton.setText(_translate("MainWindow", "Absolute Move"))
        self.groupBox_7.setTitle(_translate("MainWindow", "Command Words 2 to 9"))
        self.label_3.setText(_translate("MainWindow", "Acceleration (1-5.000)"))
        self.label.setText(_translate("MainWindow", "Position (+/- 8.388.607)"))
        self.label_2.setText(_translate("MainWindow", "Speed (1-2.999.999)"))
        self.label_4.setText(_translate("MainWindow", "Deceleration (1-5.000)"))
        self.label_9.setText(_translate("MainWindow", "Dwell Delay (0-65535)ms"))
        self.label_10.setText(_translate("MainWindow", "Jerk (0-5000)"))
        self.groupBox_8.setTitle(_translate("MainWindow", "Position"))
        self.label_11.setText(_translate("MainWindow", "Current Pos:"))
        self.label_13.setText(_translate("MainWindow", "Encoder Pos:"))
        self.capturedEncoderPositionLabel.setText(_translate("MainWindow", "0"))
        self.currentPositionLabel.setText(_translate("MainWindow", "0"))
        self.encoderPositionLabel.setText(_translate("MainWindow", "0"))
        self.label_5.setText(_translate("MainWindow", "Motor Current:"))
        self.label_15.setText(_translate("MainWindow", "Captured Enc. Pos:"))
        self.label_6.setText(_translate("MainWindow", "Acceleration Jerk:"))
        self.motorCurrentLabel.setText(_translate("MainWindow", "0"))
        self.accelerationJerkLabel.setText(_translate("MainWindow", "0"))
        self.groupBox_4.setTitle(_translate("MainWindow", "Status Word 2"))
        self.disableDriverButton.setText(_translate("MainWindow", "Enable Power"))
        self.configureButton.setText(_translate("MainWindow", "Configure Device"))
        self.actionConfiguration.setText(_translate("MainWindow", "Configuration"))
