from functools import partial
import imp
from PyQt5.QtWidgets import QDialog, QLabel, QCheckBox, QHBoxLayout, QWidget, QGridLayout
from PyQt5.QtCore import QEventLoop, Qt
from pyqtgraph.Qt import QtGui, QtCore, QtWidgets
from utils.driver_fingers import instrument_dicts

from views.move_edit_window import Ui_Dialog as MoveDialog
#from views.stay_edit_window import Ui_Dialog as StayDialog
#from views.start_edit_window import Ui_Dialog as StartDialog
from views.calibrate_angle_autohome import Ui_Dialog as CalibrateAngleDialog
from views.calibrate_flute_menu import Ui_Dialog as CalibrateFlutePosDialog
from views.dialog_control import Ui_Dialog as ConfigureFluteControlDialog
from views.start_edit_window import Ui_Dialog as StartDialog
from views.finger_window import Ui_Dialog as FingerDialog
from views.instrument_window import Ui_Dialog as InstrumentDialog
from views.score_speed_change import Ui_Dialog as ScoreSpeedDialog
from views.score_param_correction import Ui_Dialog as ScoreParamCorrectionDialog
from views.states_from_notes_form import Ui_Dialog as StatesFromNotesDialog
from views.zoom_form import Ui_Dialog as ZoomScoreDialog
from view_control.plot_pyqt import RouteWidget, RampWidget
from PyQt5.QtWidgets import QMessageBox
from PyQt5 import QtWidgets


class ConfigureFluteControlForm(QDialog, ConfigureFluteControlDialog):
    def __init__(self, parent=None, data=[0,0,0,0,0]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        self.var_comboBox.setCurrentIndex(data[0])
        self.loop_comboBox.setCurrentIndex(data[1])

        self.kp_spinbox.setValue(data[2])
        self.ki_spinbox.setValue(data[3])
        self.kd_spinbox.setValue(data[4])

        self.var_comboBox.currentIndexChanged.connect(self.change_var)
        self.loop_comboBox.currentIndexChanged.connect(self.change_loop)

        self.kp_spinbox.valueChanged.connect(self.change_kp)
        self.ki_spinbox.valueChanged.connect(self.change_ki)
        self.kd_spinbox.valueChanged.connect(self.change_kd)

    def change_var(self, value):
        self.data[0] = value
    
    def change_loop(self, value):
        self.data[1] = value
    
    def change_kp(self, value):
        self.data[2] = value
    
    def change_ki(self, value):
        self.data[3] = value
    
    def change_kd(self, value):
        self.data[4] = value


class CalibrateAngleForm(QDialog, CalibrateAngleDialog):
    def __init__(self, parent=None, data=[0]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        self.angleSpinBox.setValue(data[0])

        self.angleSpinBox.valueChanged.connect(self.change_angle)

    def change_angle(self, value):
        self.data[0] = value


class CalibrateFluteForm(QDialog, CalibrateFlutePosDialog):
    def __init__(self, parent=None, data=[0,0]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        self.fXPosSpinBox.setValue(data[0])
        self.fZPosSpinBox.setValue(data[1])

        self.fXPosSpinBox.valueChanged.connect(self.change_f_x)
        self.fZPosSpinBox.valueChanged.connect(self.change_f_z)

    def change_f_x(self, value):
        self.data[0] = value

    def change_f_z(self, value):
        self.data[1] = value


class StartActionForm(QDialog, StartDialog):
    def __init__(self, parent=None, data={'type': 0, 'r': 0, 'theta': 0,'offset': 0}):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        self.radiusSpinBox.setValue(data['r'])
        self.thetaSpinBox.setValue(data['theta'])
        self.offsetSpinBox.setValue(data['offset'])

        self.radiusSpinBox.valueChanged.connect(self.change_radius)
        self.offsetSpinBox.valueChanged.connect(self.change_offset)
        self.thetaSpinBox.valueChanged.connect(self.change_theta)

    def change_radius(self, value):
        self.data['r'] = value

    def change_offset(self, value):
        self.data['offset'] = value
    
    def change_theta(self, value):
        self.data['theta'] = value


class FingersActionForm(QDialog, FingerDialog):
    def __init__(self, parent=None, data={'time': 1, 'note': 'C4'}, index=-1, instrument='flute'):
        super().__init__(parent) # super(Form, self).__init__(parent)
        self.setupUi(self, instrument)
        self.parent = parent
        self.data = data
        self.index = index

        self.noteComboBox.setCurrentIndex(self.noteComboBox.findText(data['note']))
        self.durationSpinBox.setValue(data['time'])

        self.noteComboBox.currentIndexChanged.connect(partial(self.update_data, 'note'))
        self.durationSpinBox.valueChanged.connect(partial(self.update_data, 'time'))

    def update_data(self, tag, value):
        if tag == 'note':
            self.data['note'] = self.noteComboBox.itemText(value)
        elif tag == 'time':
            self.data['time'] = value


class MoveActionForm(QDialog, MoveDialog):
    def __init__(self, parent=None, data={'type': 0, 'move': 0, 'time': 1.0, 'r': 0, 'theta': 0, 'offset': 0, 'jerk': 0,
                                          'acceleration': 0, 'deceleration': 0, 'flow': 0, 'deformation': 0,
                                          'vibrato_amp': 0, 'vibrato_freq': 0}, index=-1):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data
        self.index = index
        self.ramp_window = None
        self.route_window = None

        self.moveParametersGroupBox.hide()
        self.resize(400,100)
        self.min_size = self.size()

        if self.data['move']:
            self.moveParametersGroupBox.show()
            
        self.durationSpinBox.setValue(data['time'])
        self.typeComboBox.setCurrentIndex(data['move'])

        self.durationSpinBox.valueChanged.connect(self.change_duration)
        self.typeComboBox.currentIndexChanged.connect(self.change_move)

        self.radiusSpinBox.setValue(data['r'])
        self.thetaSpinBox.setValue(data['theta'])
        self.offsetSpinBox.setValue(data['offset'])

        self.radiusSpinBox.valueChanged.connect(self.change_radius)
        self.thetaSpinBox.valueChanged.connect(self.change_theta)
        self.offsetSpinBox.valueChanged.connect(self.change_offset)

        self.jerkSpinBox.setValue(data['jerk'])
        self.accelerationValueSpinBox.setValue(data['acceleration'])
        self.decelerationValueSpinBox.setValue(data['deceleration'])

        self.jerkSpinBox.valueChanged.connect(self.change_jerk)
        self.accelerationValueSpinBox.valueChanged.connect(self.change_acceleration)
        self.decelerationValueSpinBox.valueChanged.connect(self.change_deceleration)

        self.flowSpinBox.setValue(data['flow'])
        self.deformationSpinBox.setValue(data['deformation'])
        self.vibratoAmplitudeSpinBox.setValue(data['vibrato_amp'])
        self.vibratoFrequencySpinBox.setValue(data['vibrato_freq'])
        #self.delayFlowSpinBox.setValue(data['delay'])
        #self.leadFlowSpinBox.setValue(data['lead'])

        self.flowSpinBox.valueChanged.connect(self.change_flow)
        self.deformationSpinBox.valueChanged.connect(self.change_deformation)
        self.vibratoAmplitudeSpinBox.valueChanged.connect(self.change_vibrato_amplitude)
        self.vibratoFrequencySpinBox.valueChanged.connect(self.change_vibrato_frequency)
        #self.delayFlowSpinBox.valueChanged.connect(self.change_delay_flow)
        #self.leadFlowSpinBox.valueChanged.connect(self.change_lead_flow)

        self.plotMovementButton.clicked.connect(self.plot_movement)
        self.plotFlowRampButton.clicked.connect(self.plot_flow_ramp)

        self.buttonBox.clicked.connect(self.close)

    def close(self):
        if self.ramp_window:
            self.ramp_window.close()
        if self.route_window:
            self.route_window.close()

    def change_duration(self, value):
        self.data['time'] = value

    def change_move(self, value):
        self.data['move'] = value
        if value == 1:
            self.moveParametersGroupBox.show()
            #self.resize(600,600)
        else:
            self.moveParametersGroupBox.hide()
            self.adjustSize()
            self.resize(self.min_size)

    def change_radius(self, value):
        self.data['r'] = value

    def change_theta(self, value):
        self.data['theta'] = value
    
    def change_offset(self, value):
        self.data['offset'] = value

    def change_jerk(self, value):
        self.data['jerk'] = value

    def change_acceleration(self, value):
        self.data['acceleration'] = value

    def change_deceleration(self, value):
        self.data['deceleration'] = value

    def change_flow(self, value):
        self.data['flow'] = value
    
    def change_deformation(self, value):
        self.data['deformation'] = value

    def change_vibrato_amplitude(self, value):
        self.data['vibrato_amp'] = value
    
    def change_vibrato_frequency(self, value):
        self.data['vibrato_freq'] = value
    
    # def change_delay_flow(self, value):
    #     self.data['delay'] = value
    
    # def change_lead_flow(self, value):
    #     self.data['lead'] = value


    def plot_movement(self):
        ri, thetai, oi, fi, v_a, v_f = self.get_last_pos()
        rf = self.data['r']
        thetaf = self.data['theta']
        of = self.data['offset']

        try:
            if not self.route_window:
                self.route_window = RouteWidget(ri, thetai, oi, rf, thetaf, of, parent=self)
                self.route_window.show()
            else:
                self.route_window.redraw(ri, thetai, oi, rf, thetaf, of)
        except:
            msg = QMessageBox()
            msg.setText("Invalid Position")
            msg.setInformativeText("It is not possible to reach such position.")
            msg.setWindowTitle("Error")
            ok_button = msg.addButton("OK", QtWidgets.QMessageBox.NoRole)

            retval = msg.exec_()

        #plan_route_min_error(ri, thetai, oi, rf, thetaf, of, plot=True)
        #w = MPLWindow(parent=self)
        #w.show()
        #print('TO-DO: Plot movement')

    def plot_flow_ramp(self):
        ri, thetai, oi, fi, v_a, v_f = self.get_last_pos()
        if not self.ramp_window:
            self.ramp_window = RampWidget(fi, self.data['flow'], self.data['vibrato_amp'], self.data['vibrato_freq'], self.data['deformation'], self.data['time'], parent=self)
            self.ramp_window.show()
        else:
            self.ramp_window.redraw(self.data['flow'], self.data['vibrato_amp'], self.data['vibrato_freq'], self.data['deformation'], self.data['time'])

    def get_last_pos(self):
        return self.parent.get_previous_pos(self.index)


class InstrumentForm(QDialog, InstrumentDialog):
    def __init__(self, parent=None, instrument='flute'):
        super().__init__(parent)
        self.setupUi(self)
        self.parent = parent

        self.instrument = instrument

        # Agrega las acciones disponibles al Combo Box
        self.comboBox.addItems([name.capitalize() for name in instrument_dicts.keys()])
        self.comboBox.setCurrentIndex(0)

        # Conecta el cambio de valor con la función correspondiente
        # self.comboBox.currentIndexChanged.connect(self.change_instrument)

    def change_instrument(self, value):
        self.instrument = instrument_dicts[self.comboBox.itemText(value).lower()]


class ScaleTimeForm(QDialog, ScoreSpeedDialog):
    def __init__(self, parent=None, data=[1]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        self.scaleSpinBox.setValue(data[0])
        self.onlyNotesCheckBox.setChecked(data[1])

        self.scaleSpinBox.valueChanged.connect(self.change_scale)
        self.onlyNotesCheckBox.toggled.connect(self.change_only_notes)

    def change_scale(self, value):
        self.data[0] = value
    
    def change_only_notes(self, value):
        self.data[1] = value


class ParamCorrectionForm(QDialog, ScoreParamCorrectionDialog):
    def __init__(self, parent=None, data=[0, 0, 0, 0]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        self.RSpinBox.setValue(data[0])
        self.ThetaSpinBox.setValue(data[1])
        self.OffsetSpinBox.setValue(data[2])
        self.FlowSpinBox.setValue(data[3])

        self.RSpinBox.valueChanged.connect(self.change_r_correction)
        self.ThetaSpinBox.valueChanged.connect(self.change_theta_correction)
        self.OffsetSpinBox.valueChanged.connect(self.change_offset_correction)
        self.FlowSpinBox.valueChanged.connect(self.change_flow_correction)

    def change_r_correction(self, value):
        self.data[0] = value

    def change_theta_correction(self, value):
        self.data[1] = value

    def change_offset_correction(self, value):
        self.data[2] = value

    def change_flow_correction(self, value):
        self.data[3] = value

class NoteWidget(QWidget):
    def __init__(self, label, width, index, parent=None):
        super(NoteWidget, self).__init__(parent)
        self.label = label
        self.parent = parent
        self.performing = False
        self.width = width
        self.selected = False
        self.index = index
        self._generateUI()
    
    def _generateUI(self):
        main_layout = QGridLayout()
        #main_layout.SetFixedSize(130)
        self.setLayout(main_layout)
        self.setFixedWidth(int(100*self.width-6))
        self.blocked = False
        #self.setFixedHeight(300)
        
        self.setSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        
        title = QLabel(self.label)
        title.setWordWrap(True)
        main_layout.addWidget(title, 0, 0, 1, 3)

        self.setAutoFillBackground(True)
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.darkBlue)
        self.setPalette(p)
    
    def resizeEvent(self, event):
        radius = 10.0
        path = QtGui.QPainterPath()
        path.addRoundedRect(QtCore.QRectF(self.rect()), radius, radius)
        mask = QtGui.QRegion(path.toFillPolygon().toPolygon())
        self.setMask(mask)
        #QtGui.QMainWindow.resizeEvent(self, event)

    def mousePressEvent(self, a0: QtGui.QMouseEvent) -> None:
        if self.blocked:
            return super().mousePressEvent(a0)
        if self.selected:
            self.unselect()
        else:
            self.select()
        return super().mousePressEvent(a0)

    def select(self):
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.darkGreen)
        self.setPalette(p)
        self.selected = True
        self.parent.data[1][self.index]['selected'] = True
    
    def unselect(self):
        p = self.palette()
        p.setColor(self.backgroundRole(), Qt.darkBlue)
        self.setPalette(p)
        self.selected = False
        self.parent.data[1][self.index]['selected'] = False
    
    def block(self):
        self.blocked = True

class StatesFromNotesForm(QDialog, StatesFromNotesDialog):
    def __init__(self, parent=None, data=[0, [], 0]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        self.accelerationSpinBox.setValue(data[0])
        self.boxes = []
        for i in range(len(data[1])):
            self.boxes.append(NoteWidget(data[1][i]['note'], data[1][i]['time'], i, parent=self))
            self.notesLayout.addWidget(self.boxes[-1])
            if data[1][i]['selected']:
                self.boxes[-1].select()

        if len(data[1]):
            self.boxes[0].select()
            self.boxes[0].block()

        self.minTimeSpinBox.setValue(data[2])

        self.accelerationSpinBox.valueChanged.connect(self.change_acc)
        self.minTimeSpinBox.valueChanged.connect(self.change_min_time)
        self.button_2.clicked.connect(self.select_all)

    def change_acc(self, value):
        self.data[0] = value
    
    def change_min_time(self, value):
        self.data[2] = value
    
    def select_all(self):
        for i in self.boxes:
            i.select()

class ZoomScoreForm(QDialog, ZoomScoreDialog):
    def __init__(self, parent=None, data=[0]):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data

        self.zoomSpinBox.setValue(data[0])
        self.zoomSpinBox.valueChanged.connect(self.change_zoom_factor)
    
    def change_zoom_factor(self, value):
        self.data[0] = value