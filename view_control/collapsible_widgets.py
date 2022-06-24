from PyQt5 import QtCore, QtGui, QtWidgets
from utils.cinematica import *

class CollapsibleBox(QtWidgets.QWidget):
    def __init__(self, title="", parent=None):
        super(CollapsibleBox, self).__init__(parent)

        self.toggle_button = QtWidgets.QToolButton(
            text=title, checkable=True, checked=False
        )
        self.toggle_button.setStyleSheet("QToolButton { border: none; }")
        self.toggle_button.setToolButtonStyle(
            QtCore.Qt.ToolButtonTextBesideIcon
        )
        self.toggle_button.setArrowType(QtCore.Qt.RightArrow)
        self.toggle_button.pressed.connect(self.on_pressed)

        self.toggle_animation = QtCore.QParallelAnimationGroup(self)

        self.content_area = QtWidgets.QScrollArea(
            maximumHeight=0, minimumHeight=0
        )
        self.content_area.setSizePolicy(
            QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed
        )
        self.content_area.setFrameShape(QtWidgets.QFrame.NoFrame)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setSpacing(0)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.toggle_button)
        lay.addWidget(self.content_area)

        self.toggle_animation.addAnimation(
            QtCore.QPropertyAnimation(self, b"minimumHeight")
        )
        self.toggle_animation.addAnimation(
            QtCore.QPropertyAnimation(self, b"maximumHeight")
        )
        self.toggle_animation.addAnimation(
            QtCore.QPropertyAnimation(self.content_area, b"maximumHeight")
        )

    @QtCore.pyqtSlot()
    def on_pressed(self):
        checked = self.toggle_button.isChecked()
        self.toggle_button.setChecked(not checked)

        self.toggle_button.setArrowType(
            QtCore.Qt.DownArrow if not checked else QtCore.Qt.RightArrow
        )
        self.toggle_animation.setDirection(
            QtCore.QAbstractAnimation.Forward
            if not checked
            else QtCore.QAbstractAnimation.Backward
        )
        self.toggle_animation.start()

    def setContentLayout(self, layout):
        lay = self.content_area.layout()
        del lay
        self.content_area.setLayout(layout)
        collapsed_height = (
            self.sizeHint().height() - self.content_area.maximumHeight()
        )
        content_height = layout.sizeHint().height()
        for i in range(self.toggle_animation.animationCount()):
            animation = self.toggle_animation.animationAt(i)
            animation.setDuration(500)
            animation.setStartValue(collapsed_height)
            animation.setEndValue(collapsed_height + content_height)

        content_animation = self.toggle_animation.animationAt(
            self.toggle_animation.animationCount() - 1
        )
        content_animation.setDuration(500)
        content_animation.setStartValue(0)
        content_animation.setEndValue(content_height)

class ManualMoveCollapsibleBox(CollapsibleBox):
    def __init__(self, title="", parent=None, state=State(0,0,0,0), desired_state=State(0,0,0,0), playing=False):
        super(ManualMoveCollapsibleBox, self).__init__(title, parent)
        self.state = state
        self.desired_state = desired_state
        self.parent = parent
        self.changing_other = False
        self.playing = playing
        self.gridLayout = QtWidgets.QGridLayout()
        self.labelX = QtWidgets.QLabel("X Axis:")
        self.spinBoxX = QtWidgets.QDoubleSpinBox()
        self.spinBoxX.setMaximum(150)
        self.labelZ = QtWidgets.QLabel("Z Axis:")
        self.spinBoxZ = QtWidgets.QDoubleSpinBox()
        self.spinBoxZ.setMaximum(150)
        self.labelAngle = QtWidgets.QLabel("Angle:")
        self.spinBoxAngle = QtWidgets.QDoubleSpinBox()
        self.spinBoxAngle.setMaximum(180)
        self.spinBoxAngle.setMinimum(-180)
        self.gridLayout.addWidget(self.labelX, 0, 0, 1, 1)
        self.gridLayout.addWidget(self.spinBoxX, 0, 1, 1, 1)
        self.gridLayout.addWidget(self.labelZ, 1, 0, 1, 1)
        self.gridLayout.addWidget(self.spinBoxZ, 1, 1, 1, 1)
        self.gridLayout.addWidget(self.labelAngle, 2, 0, 1, 1)
        self.gridLayout.addWidget(self.spinBoxAngle, 2, 1, 1, 1)
        self.labelR = QtWidgets.QLabel("Radius:")
        self.spinBoxR = QtWidgets.QDoubleSpinBox()
        self.spinBoxR.setMaximum(250)
        self.labelTheta = QtWidgets.QLabel("Theta:")
        self.spinBoxTheta = QtWidgets.QDoubleSpinBox()
        self.spinBoxTheta.setMaximum(180)
        self.spinBoxTheta.setMinimum(-180)
        self.labelOffset = QtWidgets.QLabel("Jet Offset:")
        self.spinBoxOffset = QtWidgets.QDoubleSpinBox()
        self.spinBoxOffset.setMaximum(200)
        self.spinBoxOffset.setMinimum(-200)
        self.gridLayout.addWidget(self.labelR, 0, 3, 1, 1)
        self.gridLayout.addWidget(self.spinBoxR, 0, 4, 1, 1)
        self.gridLayout.addWidget(self.labelTheta, 1, 3, 1, 1)
        self.gridLayout.addWidget(self.spinBoxTheta, 1, 4, 1, 1)
        self.gridLayout.addWidget(self.labelOffset, 2, 3, 1, 1)
        self.gridLayout.addWidget(self.spinBoxOffset, 2, 4, 1, 1)
        # spacerItem = QtWidgets.QSpacerItem(100, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        # self.gridLayout.addItem(spacerItem, 0, 2, 3, 1)
        self.labelFlow = QtWidgets.QLabel("Flow:")
        self.spinBoxFlow = QtWidgets.QDoubleSpinBox()
        self.labelFlowVibrato = QtWidgets.QLabel("Vibrato Frequency:")
        self.spinBoxFlowVibrato = QtWidgets.QDoubleSpinBox()
        self.labelFlowVibratoAmplitude = QtWidgets.QLabel("Vibrato Amplitude:")
        self.spinBoxFlowVibratoAmplitude = QtWidgets.QDoubleSpinBox()
        self.spinBoxFlow.setMaximum(50)
        self.spinBoxFlowVibrato.setMaximum(7)
        
        self.gridLayout.addWidget(self.labelFlow, 0, 6, 1, 1)
        self.gridLayout.addWidget(self.spinBoxFlow, 0, 7, 1, 1)
        self.gridLayout.addWidget(self.labelFlowVibrato, 1, 6, 1, 1)
        self.gridLayout.addWidget(self.spinBoxFlowVibrato, 1, 7, 1, 1)
        self.gridLayout.addWidget(self.labelFlowVibratoAmplitude, 2, 6, 1, 1)
        self.gridLayout.addWidget(self.spinBoxFlowVibratoAmplitude, 2, 7, 1, 1)

        # spacerItem = QtWidgets.QSpacerItem(100, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        # self.gridLayout.addItem(spacerItem, 0, 5, 3, 1)
        
        self.stopButton = QtWidgets.QPushButton('Stop')
        self.stopButton.setSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout.addWidget(self.stopButton, 0, 9, 3, 1)

        # spacerItem = QtWidgets.QSpacerItem(100, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        # self.gridLayout.addItem(spacerItem, 0, 8, 3, 1)

        self.setContentLayout(self.gridLayout)

        self.spinBoxX.setValue(self.state.x)
        self.spinBoxZ.setValue(self.state.z)
        self.spinBoxAngle.setValue(self.state.alpha)
        self.spinBoxR.setValue(self.state.r)
        self.spinBoxTheta.setValue(self.state.theta)
        self.spinBoxOffset.setValue(self.state.o)
        
        self.spinBoxX.valueChanged.connect(self.change_x)
        self.spinBoxZ.valueChanged.connect(self.change_z)
        self.spinBoxAngle.valueChanged.connect(self.change_angle)
        self.spinBoxR.valueChanged.connect(self.change_r)
        self.spinBoxTheta.valueChanged.connect(self.change_theta)
        self.spinBoxOffset.valueChanged.connect(self.change_offset)
        self.spinBoxFlow.valueChanged.connect(self.change_flow)
        self.spinBoxFlowVibrato.valueChanged.connect(self.change_flow_vibrato)
        self.spinBoxFlowVibratoAmplitude.valueChanged.connect(self.change_flow_vibrato_amp)

    def set_values(self, state):
        self.changing_other = True
        self.spinBoxX.setValue(state.x)
        self.spinBoxZ.setValue(state.z)
        self.spinBoxAngle.setValue(state.alpha)
        self.spinBoxR.setValue(state.r)
        self.spinBoxTheta.setValue(state.theta)
        self.spinBoxOffset.setValue(state.o)
        self.spinBoxFlow.setValue(state.flow)
        self.spinBoxFlowVibrato.setValue(state.vibrato_freq)
        self.spinBoxFlowVibratoAmplitude.setValue(state.vibrato_amp)
        self.desired_state.change_state(state)
        self.changing_other = False

    def update_values(self):
        #print(self.state)
        self.spinBoxX.setValue(self.desired_state.x)
        self.spinBoxZ.setValue(self.desired_state.z)
        self.spinBoxAngle.setValue(self.desired_state.alpha)
        self.spinBoxR.setValue(self.desired_state.r)
        self.spinBoxTheta.setValue(self.desired_state.theta)
        self.spinBoxOffset.setValue(self.desired_state.o)
        self.spinBoxFlow.setValue(self.desired_state.flow)
        self.spinBoxFlowVibrato.setValue(self.desired_state.vibrato_freq)
        self.spinBoxFlowVibratoAmplitude.setValue(self.desired_state.vibrato_amp)
        #self.desired_state.change_state(self.state)

    def change_x(self, value):
        if not self.changing_other and not self.playing.is_set():
            #print("x", value)
            self.changing_other = True
            self.desired_state.x = value
            self.parent.musician.move_to_state(self.desired_state, onlyCartesian=True)
            self.update_values()
            #flutist.moveTo(self.state, onlyCartesian=True)
            self.changing_other = False
    
    def change_z(self, value):
        if not self.changing_other and not self.playing.is_set():
            #print("z")
            self.changing_other = True
            self.desired_state.z = value
            #print(value, self.state.z)
            self.parent.musician.move_to_state(self.desired_state, onlyCartesian=True)
            self.update_values()
            self.changing_other = False

    def change_angle(self, value):
        if not self.changing_other and not self.playing.is_set():
            #print("alpha", value)
            self.changing_other = True
            self.desired_state.alpha = value
            self.parent.musician.move_to_state(self.desired_state, onlyCartesian=True)
            self.update_values()
            self.changing_other = False

    def change_r(self, value):
        if not self.changing_other and not self.playing.is_set():
            #print("r")
            self.changing_other = True
            self.desired_state.r = value
            self.parent.musician.move_to_state(self.desired_state)
            self.update_values()
            self.changing_other = False

    def change_theta(self, value):
        if not self.changing_other and not self.playing.is_set():
            #print("theta")
            self.changing_other = True
            self.desired_state.theta = value
            self.parent.musician.move_to_state(self.desired_state)
            self.update_values()
            self.changing_other = False

    def change_offset(self, value):
        if not self.changing_other and not self.playing.is_set():
            #print("o")
            self.changing_other = True
            self.desired_state.o = value
            self.parent.musician.move_to_state(self.desired_state)
            self.update_values()
            self.changing_other = False
    
    def change_flow(self, value):
        if not self.changing_other and not self.playing.is_set():
            self.changing_other = True
            self.desired_state.flow = value
            self.parent.musician.move_to_state(self.desired_state, T=0, onlyFlow=True)
            self.changing_other = False

    def change_flow_vibrato(self, value):
        if not self.changing_other and not self.playing.is_set():
            #print("flow vibrato")
            self.changing_other = True
            self.desired_state.vibrato_freq = value
            self.parent.musician.move_to_state(self.desired_state, T=0, onlyFlow=True)
            self.changing_other = False

    def change_flow_vibrato_amp(self, value):
        if not self.changing_other and not self.playing.is_set():
            #print("flow vibrato")
            self.changing_other = True
            self.desired_state.vibrato_amp = value
            self.parent.musician.move_to_state(self.desired_state, T=0, onlyFlow=True)
            self.changing_other = False

    def disableButtons(self):
        #self.playing = True
        self.spinBoxX.setEnabled(False)
        self.spinBoxZ.setEnabled(False)
        self.spinBoxAngle.setEnabled(False)
        self.spinBoxR.setEnabled(False)
        self.spinBoxTheta.setEnabled(False)
        self.spinBoxOffset.setEnabled(False)
        self.spinBoxFlow.setEnabled(False)
        self.spinBoxFlowVibrato.setEnabled(False)
        self.spinBoxFlowVibratoAmplitude.setEnabled(False)
        self.stopButton.setEnabled(False)


    def enableButtons(self):
        self.spinBoxX.setEnabled(True)
        self.spinBoxZ.setEnabled(True)
        self.spinBoxAngle.setEnabled(True)
        self.spinBoxR.setEnabled(True)
        self.spinBoxTheta.setEnabled(True)
        self.spinBoxOffset.setEnabled(True)
        self.spinBoxFlow.setEnabled(True)
        self.spinBoxFlowVibrato.setEnabled(True)
        self.spinBoxFlowVibratoAmplitude.setEnabled(True)
        self.stopButton.setEnabled(True)
        #self.playing = False

    # def set_state(self, state):
    #     self.changing_other = True
    #     self.spinBoxX.setValue(state.x)
    #     self.spinBoxZ.setValue(state.z)
    #     self.spinBoxAngle.setValue(state.alpha)
    #     self.spinBoxR.setValue(state.r)
    #     self.spinBoxTheta.setValue(state.theta)
    #     self.spinBoxOffset.setValue(state.o)
    #     self.spinBoxFlow.setValue(state.flow)
    #     self.spinBoxFlowVibrato.setValue(0)
    #     self.changing_other = False

        # lay = QtWidgets.QVBoxLayout()
        # for j in range(8):
        #     label = QtWidgets.QLabel("{}".format(j))
        #     color = QtGui.QColor(*[random.randint(0, 255) for _ in range(3)])
        #     label.setStyleSheet(
        #         "background-color: {}; color : white;".format(color.name())
        #     )
        #     label.setAlignment(QtCore.Qt.AlignCenter)
        #     lay.addWidget(label)