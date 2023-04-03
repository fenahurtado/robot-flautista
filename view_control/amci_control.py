from functools import partial
import sys
sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
import threading
from time import sleep

from PyQt5.QtWidgets import QApplication, QHBoxLayout, QWidget

import numpy as np
import matplotlib.pyplot as plt

from view_control.LedIndicatorWidget import *
from views.amci_driver import Ui_MainWindow as AMCIWindow
from views.amci_driver_configuration import Ui_Dialog as ConfigureAMCIDriverDialog
from utils.driver_amci import INPUT_FUNCTION_BITS, AMCIDriver
from views.assembled_move_step1 import Ui_Dialog as AssembledDialog1
from views.assembled_move_step2 import Ui_Dialog as AssembledDialog2

class AssembledForm1(QDialog, AssembledDialog1):
    def __init__(self, parent=None, data={'profile': 0, 'dwell_time': 0, 'segments': 1}):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data
        self.moveProfileComboBox.setCurrentIndex(self.data['profile'])
        self.moveProfileComboBox.currentIndexChanged.connect(partial(self.update_data, 'profile'))
        self.dwellTimeSpinBox.setValue(self.data['dwell_time'])
        self.dwellTimeSpinBox.setEnabled(self.data['profile'])
        self.dwellTimeSpinBox.valueChanged.connect(partial(self.update_data, 'dwell_time'))
        self.segmentsSpinBox.setValue(self.data['segments'])
        self.segmentsSpinBox.valueChanged.connect(partial(self.update_data, 'segments'))
    
    def update_data(self, tag, value):
        self.data[tag] = value
        if tag == 'profile':
            self.dwellTimeSpinBox.setEnabled(value)



class AssembledForm2(QDialog, AssembledDialog2):
    def __init__(self, N, parent=None, data={'segment1_pos': 0, 'segment1_speed': 0, 'segment1_acceleration': 0, 'segment1_deceleration': 0, 'segment1_jerk': 0, 'segment2_pos': 0, 'segment2_speed': 0, 'segment2_acceleration': 0, 'segment2_deceleration': 0, 'segment2_jerk': 0, 'segment3_pos': 0, 'segment3_speed': 0, 'segment3_acceleration': 0, 'segment3_deceleration': 0, 'segment3_jerk': 0, 'segment4_pos': 0, 'segment4_speed': 0, 'segment4_acceleration': 0, 'segment4_deceleration': 0, 'segment4_jerk': 0, 'segment5_pos': 0, 'segment5_speed': 0, 'segment5_acceleration': 0, 'segment5_deceleration': 0, 'segment5_jerk': 0, 'segment6_pos': 0, 'segment6_speed': 0, 'segment6_acceleration': 0, 'segment6_deceleration': 0, 'segment6_jerk': 0, 'segment7_pos': 0, 'segment7_speed': 0, 'segment7_acceleration': 0, 'segment7_deceleration': 0, 'segment7_jerk': 0, 'segment8_pos': 0, 'segment8_speed': 0, 'segment8_acceleration': 0, 'segment8_deceleration': 0, 'segment8_jerk': 0, 'segment9_pos': 0, 'segment9_speed': 0, 'segment9_acceleration': 0, 'segment9_deceleration': 0, 'segment9_jerk': 0, 'segment10_pos': 0, 'segment10_speed': 0, 'segment10_acceleration': 0, 'segment10_deceleration': 0, 'segment10_jerk': 0, 'segment11_pos': 0, 'segment11_speed': 0, 'segment11_acceleration': 0, 'segment11_deceleration': 0, 'segment11_jerk': 0, 'segment12_pos': 0, 'segment12_speed': 0, 'segment12_acceleration': 0, 'segment12_deceleration': 0, 'segment12_jerk': 0, 'segment13_pos': 0, 'segment13_speed': 0, 'segment13_acceleration': 0, 'segment13_deceleration': 0, 'segment13_jerk': 0, 'segment14_pos': 0, 'segment14_speed': 0, 'segment14_acceleration': 0, 'segment14_deceleration': 0, 'segment14_jerk': 0, 'segment15_pos': 0, 'segment15_speed': 0, 'segment15_acceleration': 0, 'segment15_deceleration': 0, 'segment15_jerk': 0, 'segment16_pos': 0, 'segment16_speed': 0, 'segment16_acceleration': 0, 'segment16_deceleration': 0, 'segment16_jerk': 0}):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.N = N
        self.data = data
        self.positions = []
        self.speeds = []
        self.accelerations = []
        self.decelerations = []
        self.jerks = []
        for i in range(self.N):
            label = QLabel(f'Segment {i+1}:')
            pos   = QSpinBox()
            pos.setMaximum(8388607)
            pos.setMinimum(-8388607)
            pos.valueChanged.connect(partial(self.update_data, f'segment{i+1}_pos'))
            speed = QSpinBox()
            speed.setMaximum(2999999)
            speed.setMinimum(1)
            speed.valueChanged.connect(partial(self.update_data, f'segment{i+1}_speed'))
            accel = QSpinBox()
            accel.setMaximum(5000)
            accel.setMinimum(1)
            accel.valueChanged.connect(partial(self.update_data, f'segment{i+1}_acceleration'))
            decel = QSpinBox()
            decel.setMaximum(5000)
            decel.setMinimum(1)
            decel.valueChanged.connect(partial(self.update_data, f'segment{i+1}_deceleration'))
            jerk  = QSpinBox()
            jerk.setMaximum(5000)
            jerk.setMinimum(0)
            jerk.valueChanged.connect(partial(self.update_data, f'segment{i+1}_jerk'))
            self.inputGridLayout.addWidget(label, i+1, 0, 1, 1)
            self.inputGridLayout.addWidget(pos, i+1, 1, 1, 1)
            self.inputGridLayout.addWidget(speed, i+1, 2, 1, 1)
            self.inputGridLayout.addWidget(accel, i+1, 3, 1, 1)
            self.inputGridLayout.addWidget(decel, i+1, 4, 1, 1)
            self.inputGridLayout.addWidget(jerk, i+1, 5, 1, 1)
        self.resize(600,100 + 30*self.N)
    
    def update_data(self, tag, value):
        self.data[tag] = value

class ConfigureAMCIDriverForm(QDialog, ConfigureAMCIDriverDialog):
    def __init__(self, parent=None, data={"disable_anti_resonance_bit":0, "enable_stall_detection_bit":0, "use_backplane_proximity_bit":0, "use_encoder_bit":0, "home_to_encoder_z_pulse":0, "input_3_function_bits":0, "input_2_function_bits":0, "input_1_function_bits":0, "output_functionality_bit":0, "output_state_control_on_network_lost":0, "output_state_on_network_lost":0, "read_present_configuration":0, "save_configuration":0, "binary_input_format":0, "binary_output_format":0, "binary_endian":0, "input_3_active_level":0, "input_2_active_level":0, "input_1_active_level":0, "starting_speed":50, "motors_step_turn":1000, "hybrid_control_gain":0, "encoder_pulses_turn":1000, "idle_current_percentage":30, "motor_current":30, "current_loop_gain":5}):
        super().__init__(parent) #super(Form, self).__init__(parent)
        self.setupUi(self)
        self.parent = parent
        self.data = data
        self.fill_values()
        self.connect_signals()

    def fill_values(self):
        self.antiResonanceCheckBox.setChecked(not self.data["disable_anti_resonance_bit"])
        self.stallDetectionCheckBox.setChecked(self.data["enable_stall_detection_bit"])
        self.homeProximityCheckBox.setChecked(self.data["use_backplane_proximity_bit"])
        self.quadEncoderCheckBox.setChecked(self.data["use_encoder_bit"])
        self.homeToZCheckBox.setChecked(self.data["home_to_encoder_z_pulse"])

        self.input3FunctionComboBox.setCurrentIndex(self.data["input_3_function_bits"])
        self.input2FunctionComboBox.setCurrentIndex(self.data["input_2_function_bits"])
        self.input1FunctionComboBox.setCurrentIndex(self.data["input_1_function_bits"])

        self.output1FunctionalityFaultOutputRadioButton.setChecked(not self.data["output_functionality_bit"])
        self.output1FunctionalityGPORadioButton.setChecked(self.data["output_functionality_bit"])

        self.output1ModeLastValueRadioButton.setChecked(not self.data["output_state_control_on_network_lost"])
        self.output1ModeBit12RadioButton.setChecked(self.data["output_state_control_on_network_lost"])

        self.output1State0RadioButton.setChecked(not self.data["output_state_on_network_lost"])
        self.output1State1RadioButton.setChecked(self.data["output_state_on_network_lost"])
        
        self.input3HighRadioButton.setChecked(self.data["input_3_active_level"])
        self.input3LowRadioButton.setChecked(not self.data["input_3_active_level"])

        self.input2HighRadioButton.setChecked(self.data["input_2_active_level"])
        self.input2LowRadioButton.setChecked(not self.data["input_2_active_level"])

        self.input1HighRadioButton.setChecked(self.data["input_1_active_level"])
        self.input1LowRadioButton.setChecked(not self.data["input_1_active_level"])

        self.dataEndianLittleRadioButton.setChecked(not self.data["binary_endian"])
        self.dataEndianBigRadioButton.setChecked(self.data["binary_endian"])

        self.positionDataFormatAMCIRadioButton.setChecked(not self.data["binary_output_format"])
        self.positionDataFormatBinaryRadioButton.setChecked(self.data["binary_output_format"])

        self.saveToFlashNoRadioButton.setChecked(not self.data["save_configuration"])
        self.saveToFlashYesRadioButton.setChecked(self.data["save_configuration"])

        self.startingSpeedSpinBox.setValue(self.data["starting_speed"])
        self.motorStepsSpinBox.setValue(self.data["motors_step_turn"])
        self.hybridControlGainSpinBox.setValue(self.data["hybrid_control_gain"])
        self.encoderPulsesSpinBox.setValue(self.data["encoder_pulses_turn"])
        self.idleCurrentSpinBox.setValue(self.data["idle_current_percentage"])
        self.motorCurrentSpinBox.setValue(self.data["motor_current"])
        self.currentLoopGainSpinBox.setValue(self.data["current_loop_gain"])

    def connect_signals(self):
        self.antiResonanceCheckBox.stateChanged.connect(partial(self.update_data, "disable_anti_resonance_bit"))
        self.stallDetectionCheckBox.stateChanged.connect(partial(self.update_data, "enable_stall_detection_bit"))
        self.homeProximityCheckBox.stateChanged.connect(partial(self.update_data, "use_backplane_proximity_bit"))
        self.quadEncoderCheckBox.stateChanged.connect(partial(self.update_data, "use_encoder_bit"))
        self.homeToZCheckBox.stateChanged.connect(partial(self.update_data, "home_to_encoder_z_pulse"))

        self.input3FunctionComboBox.currentIndexChanged.connect(partial(self.update_data, "input_3_function_bits"))
        self.input2FunctionComboBox.currentIndexChanged.connect(partial(self.update_data, "input_2_function_bits"))
        self.input1FunctionComboBox.currentIndexChanged.connect(partial(self.update_data, "input_1_function_bits"))
        
        self.output1FunctionalityFaultOutputRadioButton.clicked.connect(partial(self.update_data, "output_functionality_bit", 0))
        self.output1FunctionalityGPORadioButton.clicked.connect(partial(self.update_data, "output_functionality_bit", 1))

        self.output1ModeLastValueRadioButton.clicked.connect(partial(self.update_data, "output_state_control_on_network_lost", 0))
        self.output1ModeBit12RadioButton.clicked.connect(partial(self.update_data, "output_state_control_on_network_lost", 1))

        self.output1State0RadioButton.clicked.connect(partial(self.update_data, "output_state_on_network_lost", 0))
        self.output1State1RadioButton.clicked.connect(partial(self.update_data, "output_state_on_network_lost", 1))

        self.input3LowRadioButton.clicked.connect(partial(self.update_data, "input_3_active_level", 0))
        self.input3HighRadioButton.clicked.connect(partial(self.update_data, "input_3_active_level", 1))

        self.input2LowRadioButton.clicked.connect(partial(self.update_data, "input_2_active_level", 0))
        self.input2HighRadioButton.clicked.connect(partial(self.update_data, "input_2_active_level", 1))

        self.input1LowRadioButton.clicked.connect(partial(self.update_data, "input_1_active_level", 0))
        self.input1HighRadioButton.clicked.connect(partial(self.update_data, "input_1_active_level", 1))

        self.dataEndianLittleRadioButton.clicked.connect(partial(self.update_data, "binary_endian", 0))
        self.dataEndianBigRadioButton.clicked.connect(partial(self.update_data, "binary_endian", 1))

        self.positionDataFormatAMCIRadioButton.clicked.connect(partial(self.update_data, "binary_output_format", 0))
        self.positionDataFormatBinaryRadioButton.clicked.connect(partial(self.update_data, "binary_output_format", 1))

        self.saveToFlashNoRadioButton.clicked.connect(partial(self.update_data, "save_configuration", 0))
        self.saveToFlashYesRadioButton.clicked.connect(partial(self.update_data, "save_configuration", 1))

        self.startingSpeedSpinBox.valueChanged.connect(partial(self.update_data, "starting_speed"))
        self.motorStepsSpinBox.valueChanged.connect(partial(self.update_data, "motors_step_turn"))
        self.hybridControlGainSpinBox.valueChanged.connect(partial(self.update_data, "hybrid_control_gain"))
        self.encoderPulsesSpinBox.valueChanged.connect(partial(self.update_data, "encoder_pulses_turn"))
        self.idleCurrentSpinBox.valueChanged.connect(partial(self.update_data, "idle_current_percentage"))
        self.motorCurrentSpinBox.valueChanged.connect(partial(self.update_data, "motor_current"))
        self.currentLoopGainSpinBox.valueChanged.connect(partial(self.update_data, "current_loop_gain"))

    def update_data(self, tag, value):
        if tag == "disable_anti_resonance_bit":
            self.data[tag] = int(value == 0)
        elif tag in ["use_backplane_proximity_bit", "enable_stall_detection_bit", "use_encoder_bit", "home_to_encoder_z_pulse"]:
            self.data[tag] = int(value > 0)
        else:
            self.data[tag] = value

class AMCIWidget(QMainWindow, AMCIWindow):
    def __init__(self, driver, parent=None):
        super().__init__(parent)
        self.setupUi(self)

        self.driver = driver
        self.add_indicators()
        self.connect_signals()
        #self.driver.start()

    def add_indicators(self):
        self.module_ok_status = LedWidget('Module OK', self, color='g')
        self.module_ok_status.led.setDisabled(True)
        self.module_ok_status.led.setDisabled(self.driver.module_ok)
        self.commandStatus0Layout.addWidget(self.module_ok_status, 0, 0, 1, 1)
        self.driver.module_ok_signal.connect(self.switch_led_module_ok)

        self.configuration_error_status = LedWidget('Configuration Error', self, color='r')
        self.configuration_error_status.led.setDisabled(True)
        self.configuration_error_status.led.setChecked(self.driver.configuration_error)
        self.commandStatus0Layout.addWidget(self.configuration_error_status, 1, 0, 1, 1)
        self.driver.configuration_error_signal.connect(self.switch_configuration_error)

        self.command_error_status = LedWidget('Command Error', self, color='r')
        self.command_error_status.led.setDisabled(True)
        self.command_error_status.led.setChecked(self.driver.command_error)
        self.commandStatus0Layout.addWidget(self.command_error_status, 2, 0, 1, 1)
        self.driver.command_error_signal.connect(self.switch_command_error)

        self.input_error_status = LedWidget('Input Error', self, color='r')
        self.input_error_status.led.setDisabled(True)
        self.input_error_status.led.setChecked(self.driver.input_error)
        self.commandStatus0Layout.addWidget(self.input_error_status, 0, 1, 1, 1)
        self.driver.input_error_signal.connect(self.switch_input_error)

        self.position_invalid = False
        self.position_invalid_status = LedWidget('Position Invalid', self, color='r')
        self.position_invalid_status.led.setDisabled(True)
        self.position_invalid_status.led.setChecked(self.driver.position_invalid)
        self.commandStatus0Layout.addWidget(self.position_invalid_status, 1, 1, 1, 1)
        self.driver.position_invalid_signal.connect(self.switch_position_invalid)

        self.waiting_for_assembled_segment_status = LedWidget('Waiting Assembled', self, color='g')
        self.waiting_for_assembled_segment_status.led.setDisabled(True)
        self.waiting_for_assembled_segment_status.led.setChecked(self.driver.waiting_for_assembled_segment)
        self.commandStatus0Layout.addWidget(self.waiting_for_assembled_segment_status, 2, 1, 1, 1)
        self.waiting_for_assembled = False
        self.driver.waiting_for_assembled_segment_signal.connect(self.switch_waiting_for_assembled_segment)

        self.in_assembled_mode_status = LedWidget('Assembled mode', self, color='g')
        self.in_assembled_mode_status.led.setDisabled(True)
        self.in_assembled_mode_status.led.setChecked(self.driver.in_assembled_mode)
        self.commandStatus0Layout.addWidget(self.in_assembled_mode_status, 0, 2, 1, 1)
        self.driver.in_assembled_mode_signal.connect(self.switch_in_assembled_mode)

        self.move_complete_status = LedWidget('Move Complete', self, color='g')
        self.move_complete_status.led.setDisabled(True)
        self.move_complete_status.led.setChecked(self.driver.move_complete)
        self.commandStatus0Layout.addWidget(self.move_complete_status, 1, 2, 1, 1)
        self.driver.move_complete_signal.connect(self.switch_move_complete)

        self.decelerating_status = LedWidget('Decelerating', self, color='g')
        self.decelerating_status.led.setDisabled(True)
        self.decelerating_status.led.setChecked(self.driver.decelerating)
        self.commandStatus0Layout.addWidget(self.decelerating_status, 2, 2, 1, 1)
        self.driver.decelerating_signal.connect(self.switch_decelerating)

        self.accelerating_status = LedWidget('Accelerating', self, color='g')
        self.accelerating_status.led.setDisabled(True)
        self.accelerating_status.led.setChecked(self.driver.accelerating)
        self.commandStatus0Layout.addWidget(self.accelerating_status, 0, 3, 1, 1)
        self.driver.accelerating_signal.connect(self.switch_accelerating)

        self.at_home_status = LedWidget('At Home', self, color='g')
        self.at_home_status.led.setDisabled(True)
        self.at_home_status.led.setChecked(self.driver.at_home)
        self.commandStatus0Layout.addWidget(self.at_home_status, 1, 3, 1, 1)
        self.driver.at_home_signal.connect(self.switch_at_home)

        self.stopped_status = LedWidget('Stopped', self, color='g')
        self.stopped_status.led.setDisabled(True)
        self.stopped_status.led.setChecked(self.driver.stopped)
        self.commandStatus0Layout.addWidget(self.stopped_status, 2, 3, 1, 1)
        self.driver.stopped_signal.connect(self.switch_stopped)

        self.in_hold_state_status = LedWidget('In Hold', self, color='g')
        self.in_hold_state_status.led.setDisabled(True)
        self.in_hold_state_status.led.setChecked(self.driver.in_hold_state)
        self.commandStatus0Layout.addWidget(self.in_hold_state_status, 0, 4, 1, 1)
        self.driver.in_hold_state_signal.connect(self.switch_in_hold_state)

        self.moving_ccw_status = LedWidget('Moving CCW', self, color='g')
        self.moving_ccw_status.led.setDisabled(True)
        self.moving_ccw_status.led.setDisabled(self.driver.moving_ccw)
        self.commandStatus0Layout.addWidget(self.moving_ccw_status, 1, 4, 1, 1)
        self.driver.moving_ccw_signal.connect(self.switch_moving_ccw)

        self.moving_cw_status = LedWidget('Moving CW', self, color='g')
        self.moving_cw_status.led.setDisabled(True)
        self.moving_cw_status.led.setChecked(self.driver.moving_cw)
        self.commandStatus0Layout.addWidget(self.moving_cw_status, 2, 4, 1, 1)
        self.driver.moving_cw_signal.connect(self.switch_moving_cw)

        self.driver_is_enabled = False
        self.driver_is_enabled_status = LedWidget('Driver Enabled', self, color='g')
        self.driver_is_enabled_status.led.setDisabled(True)
        self.driver_is_enabled_status.led.setChecked(self.driver.driver_is_enabled)
        self.commandStatus1Layout.addWidget(self.driver_is_enabled_status, 0, 0, 1, 1)
        self.driver.driver_is_enabled_signal.connect(self.switch_driver_is_enabled)

        self.stall_detected_status = LedWidget('Stall Detected', self, color='r')
        self.stall_detected_status.led.setDisabled(True)
        self.stall_detected_status.led.setChecked(self.driver.stall_detected)
        self.commandStatus1Layout.addWidget(self.stall_detected_status, 1, 0, 1, 1)
        self.driver.stall_detected_signal.connect(self.switch_stall_detected)

        self.output_state_status = LedWidget('Output State', self, color='g')
        self.output_state_status.led.setDisabled(True)
        self.output_state_status.led.setChecked(self.driver.output_state)
        self.commandStatus1Layout.addWidget(self.output_state_status, 2, 0, 1, 1)
        self.driver.output_state_signal.connect(self.switch_output_state)

        self.heartbeat_bit_status = LedWidget('Heartbeat Status', self, color='y')
        self.heartbeat_bit_status.led.setDisabled(True)
        self.heartbeat_bit_status.led.setChecked(self.driver.heartbeat_bit)
        self.commandStatus1Layout.addWidget(self.heartbeat_bit_status, 3, 0, 1, 1)
        self.driver.heartbeat_bit_signal.connect(self.switch_heartbeat_bit)

        self.limit_condition_status = LedWidget('Limit Condition', self, color='g')
        self.limit_condition_status.led.setDisabled(True)
        self.limit_condition_status.led.setChecked(self.driver.limit_condition)
        self.commandStatus1Layout.addWidget(self.limit_condition_status, 4, 0, 1, 1)
        self.driver.limit_condition_signal.connect(self.switch_limit_condition)

        self.invalid_jog_change_status = LedWidget('Invalid Jog', self, color='r')
        self.invalid_jog_change_status.led.setDisabled(True)
        self.invalid_jog_change_status.led.setChecked(self.driver.invalid_jog_change)
        self.commandStatus1Layout.addWidget(self.invalid_jog_change_status, 0, 1, 1, 1)
        self.driver.invalid_jog_change_signal.connect(self.switch_invalid_jog_change)

        self.motion_lag_status = LedWidget('Motion Lag', self, color='g')
        self.motion_lag_status.led.setDisabled(True)
        self.motion_lag_status.led.setChecked(self.driver.motion_lag)
        self.commandStatus1Layout.addWidget(self.motion_lag_status, 1, 1, 1, 1)
        self.driver.motion_lag_signal.connect(self.switch_motion_lag)

        self.driver_fault_status = LedWidget('Driver Fault', self, color='r')
        self.driver_fault_status.led.setDisabled(True)
        self.driver_fault_status.led.setChecked(self.driver.driver_fault)
        self.commandStatus1Layout.addWidget(self.driver_fault_status, 2, 1, 1, 1)
        self.driver.driver_fault_signal.connect(self.switch_driver_fault)

        self.connection_was_lost_status = LedWidget('Connection Was Lost', self, color='r')
        self.connection_was_lost_status.led.setDisabled(True)
        self.connection_was_lost_status.led.setChecked(self.driver.connection_was_lost)
        self.commandStatus1Layout.addWidget(self.connection_was_lost_status, 3, 1, 1, 1)
        self.driver.connection_was_lost_signal.connect(self.switch_connection_was_lost)

        self.plc_in_prog_mode_status = LedWidget('PLC in Prog', self, color='g')
        self.plc_in_prog_mode_status.led.setDisabled(True)
        self.plc_in_prog_mode_status.led.setChecked(self.driver.plc_in_prog_mode)
        self.commandStatus1Layout.addWidget(self.plc_in_prog_mode_status, 4, 1, 1, 1)
        self.driver.plc_in_prog_mode_signal.connect(self.switch_plc_in_prog_mode)

        self.temperature_above_90_status = LedWidget('Temperature', self, color='r')
        self.temperature_above_90_status.led.setDisabled(True)
        self.temperature_above_90_status.led.setChecked(self.driver.temperature_above_90)
        self.commandStatus1Layout.addWidget(self.temperature_above_90_status, 0, 2, 1, 1)
        self.driver.temperature_above_90_signal.connect(self.switch_temperature_above_90)

        self.in_3_active_status = LedWidget('Input 3 Active', self, color='g')
        self.in_3_active_status.led.setDisabled(True)
        self.in_3_active_status.led.setChecked(self.driver.in_3_active)
        self.commandStatus1Layout.addWidget(self.in_3_active_status, 1, 2, 1, 1)
        self.driver.in_3_active_signal.connect(self.switch_in_3_active)

        self.in_2_active_status = LedWidget('Input 2 Active', self, color='g')
        self.in_2_active_status.led.setDisabled(True)
        self.in_2_active_status.led.setChecked(self.driver.in_2_active)
        self.commandStatus1Layout.addWidget(self.in_2_active_status, 2, 2, 1, 1)
        self.driver.in_2_active_signal.connect(self.switch_in_2_active)

        self.in_1_active_status = LedWidget('Input 1 Active', self, color='g')
        self.in_1_active_status.led.setDisabled(True)
        self.in_1_active_status.led.setChecked(self.driver.in_1_active)
        self.commandStatus1Layout.addWidget(self.in_1_active_status, 3, 2, 1, 1)
        self.driver.in_1_active_signal.connect(self.switch_in_1_active)

        self.motor_position = 0
        self.driver.motor_position_signal.connect(self.change_motor_position)
        self.encoder_position = 0
        self.driver.encoder_position_signal.connect(self.change_encoder_position)
        self.captured_encoder_position = 0
        self.driver.captured_encoder_position_signal.connect(self.change_captured_encoder_position)
        self.programed_motor_current = 0
        self.driver.programed_motor_current_signal.connect(self.change_programed_motor_current)
        self.accelerating_jerk = 0
        self.driver.acceleration_jerk_signal.connect(self.change_acceleration_jerk)

    def connect_signals(self):
        self.configureButton.clicked.connect(self.open_configuration)
        self.disableDriverButton.clicked.connect(self.enable_disable_driver)
        self.resetErrorsButton.clicked.connect(self.reset_errors)
        self.position = 0
        self.positionSpinBox.valueChanged.connect(self.change_position)
        self.presetPositionButton.clicked.connect(self.preset_position)
        self.speed = 200
        self.speedSpinBox.valueChanged.connect(self.change_speed)
        self.acceleration = 100
        self.accelerationSpinBox.valueChanged.connect(self.change_acceleration)
        self.deceleration = 100
        self.decelerationSpinBox.valueChanged.connect(self.change_deceleration)
        self.dwell_delay = 0
        self.dwellDelaySpinBox.valueChanged.connect(self.change_dwell_delay)
        self.jerk = 0
        self.jerkSpinBox.valueChanged.connect(self.change_jerk)
        # self.synchrostep_moving = False
        # self.synchrostepMoveButton.clicked.connect(self.synchrostep_move)
        self.manual_ccw_moving = False
        self.manualCCWButton.clicked.connect(self.manual_ccw)
        self.manual_cw_moving = False
        self.manualCWButton.clicked.connect(self.manual_cw)
        self.relative_moving = False
        self.relativeMoveButton.clicked.connect(self.relative_move)
        self.absolute_moving = False
        self.absoluteMoveButton.clicked.connect(self.absolute_move)
        self.absolute_moving_on_hold = False
        self.relative_moving_on_hold = False
        self.manual_ccw_on_hold = False
        self.manual_cw_on_hold = False
        self.assembly_moving_on_hold = False
        self.holdMoveButton.clicked.connect(self.hold_move)
        self.resumeMoveButton.clicked.connect(self.resume)
        self.immediateStopButton.clicked.connect(self.immediate_stop)
        self.ccw_finding_home = False
        self.homeCCWButton.clicked.connect(self.home_ccw)
        self.cw_finding_home = False
        self.homeCWButton.clicked.connect(self.home_cw)
        self.programming_assembly = False
        self.segment_assembly_index = 0
        self.prgAssemblyButton.clicked.connect(self.program_assembly)
        self.dwell_move = False
        self.dwellMoveCheckBox.stateChanged.connect(self.change_dwell_move)
        self.reverse_blend_dir = False
        self.blendDirCheckBox.stateChanged.connect(self.change_blend_dir)
        self.presetEncoderButton.clicked.connect(self.preset_encoder)
        self.running_assembly = False
        self.runAssemblyButton.clicked.connect(self.run_assembly)
    
    def change_blend_dir(self, value):
        self.reverse_blend_dir = value>0


    def preset_encoder(self):
        self.driver.request_write_preset_encoder_position(self.position)

    def change_dwell_move(self, value):
        self.dwell_move = int(value>0)

    def program_assembly(self):
        self.assembly_data1={'profile': self.dwell_move, 'dwell_time': self.dwell_delay, 'segments': 1}
        dlg = AssembledForm1(parent=self, data=self.assembly_data1)
        dlg.setWindowTitle("Assembly Prog. 1")
        rsp = dlg.exec()
        if rsp:
            self.assembly_data2={'segment1_pos': 0, 'segment1_speed': 0, 'segment1_acceleration': 0, 'segment1_deceleration': 0, 'segment1_jerk': 0, 'segment2_pos': 0, 'segment2_speed': 0, 'segment2_acceleration': 0, 'segment2_deceleration': 0, 'segment2_jerk': 0, 'segment3_pos': 0, 'segment3_speed': 0, 'segment3_acceleration': 0, 'segment3_deceleration': 0, 'segment3_jerk': 0, 'segment4_pos': 0, 'segment4_speed': 0, 'segment4_acceleration': 0, 'segment4_deceleration': 0, 'segment4_jerk': 0, 'segment5_pos': 0, 'segment5_speed': 0, 'segment5_acceleration': 0, 'segment5_deceleration': 0, 'segment5_jerk': 0, 'segment6_pos': 0, 'segment6_speed': 0, 'segment6_acceleration': 0, 'segment6_deceleration': 0, 'segment6_jerk': 0, 'segment7_pos': 0, 'segment7_speed': 0, 'segment7_acceleration': 0, 'segment7_deceleration': 0, 'segment7_jerk': 0, 'segment8_pos': 0, 'segment8_speed': 0, 'segment8_acceleration': 0, 'segment8_deceleration': 0, 'segment8_jerk': 0, 'segment9_pos': 0, 'segment9_speed': 0, 'segment9_acceleration': 0, 'segment9_deceleration': 0, 'segment9_jerk': 0, 'segment10_pos': 0, 'segment10_speed': 0, 'segment10_acceleration': 0, 'segment10_deceleration': 0, 'segment10_jerk': 0, 'segment11_pos': 0, 'segment11_speed': 0, 'segment11_acceleration': 0, 'segment11_deceleration': 0, 'segment11_jerk': 0, 'segment12_pos': 0, 'segment12_speed': 0, 'segment12_acceleration': 0, 'segment12_deceleration': 0, 'segment12_jerk': 0, 'segment13_pos': 0, 'segment13_speed': 0, 'segment13_acceleration': 0, 'segment13_deceleration': 0, 'segment13_jerk': 0, 'segment14_pos': 0, 'segment14_speed': 0, 'segment14_acceleration': 0, 'segment14_deceleration': 0, 'segment14_jerk': 0, 'segment15_pos': 0, 'segment15_speed': 0, 'segment15_acceleration': 0, 'segment15_deceleration': 0, 'segment15_jerk': 0, 'segment16_pos': 0, 'segment16_speed': 0, 'segment16_acceleration': 0, 'segment16_deceleration': 0, 'segment16_jerk': 0}
            dlg = AssembledForm2(parent=self, N=self.assembly_data1['segments'], data=self.assembly_data2)
            dlg.setWindowTitle("Assembly Prog. 2")
            rsp = dlg.exec()
            if rsp:
                self.dwellMoveCheckBox.setChecked(self.assembly_data1['profile'])
                self.dwellDelaySpinBox.setValue(self.assembly_data1['dwell_time'])
                self.programming_assembly = True
                self.segment_assembly_index = 0
                self.driver.request_write_program_assembled()
                # for i in range(self.assembly_data1['segments']):
                #     print(self.assembly_data2[f'segment{i+1}_pos'], self.assembly_data2[f'segment{i+1}_speed'], self.assembly_data2[f'segment{i+1}_acceleration'], self.assembly_data2[f'segment{i+1}_deceleration'], self.assembly_data2[f'segment{i+1}_jerk'])
                    #self.driver.request_write_assembled_segment(target_position=self.assembly_data2[f'segment{i+1}_pos'], programmed_speed=self.assembly_data2[f'segment{i+1}_speed'], acceleration=self.assembly_data2[f'segment{i+1}_acceleration'], deceleration=self.assembly_data2[f'segment{i+1}_deceleration'], acceleration_jerk=self.assembly_data2[f'segment{i+1}_jerk'])
                    #self.driver.request_write_program_assembled()
                #self.driver.request_write_return_to_command_mode()
                    
    def run_assembly(self):
        self.absolute_moving_on_hold = False
        self.relative_moving_on_hold = False
        self.manual_ccw_on_hold = False
        self.manual_cw_on_hold = False
        if self.running_assembly:
            self.running_assembly = False
            self.relativeMoveButton.setEnabled(True)
            self.presetEncoderButton.setEnabled(True)
            self.runAssemblyButton.setEnabled(True)
            self.prgAssemblyButton.setEnabled(True)
            self.resetErrorsButton.setEnabled(True)
            self.presetPositionButton.setEnabled(True)
            self.manualCWButton.setEnabled(True)
            self.homeCCWButton.setEnabled(True)
            self.homeCWButton.setEnabled(True)
            self.resumeMoveButton.setEnabled(True)
            self.manualCCWButton.setEnabled(True)
            self.absoluteMoveButton.setEnabled(not self.position_invalid)
            self.driver.request_write_return_to_command_mode()
        else:
            self.running_assembly = True
            # self.relativeMoveButton.setEnabled(False)
            # self.presetEncoderButton.setEnabled(False)
            # self.runAssemblyButton.setEnabled(False)
            # self.prgAssemblyButton.setEnabled(False)
            # self.resetErrorsButton.setEnabled(False)
            # self.presetPositionButton.setEnabled(False)
            # self.manualCWButton.setEnabled(False)
            # self.homeCCWButton.setEnabled(False)
            # self.homeCWButton.setEnabled(False)
            # self.resumeMoveButton.setEnabled(False)
            # self.manualCCWButton.setEnabled(False)
            # self.absoluteMoveButton.setEnabled(False)
            self.driver.request_write_run_assembled_move(blend_direction=int(self.reverse_blend_dir), dwell_move=int(self.dwell_move), dwell_time=self.dwell_delay)

    def open_configuration(self):
        data = {"disable_anti_resonance_bit":self.driver.disable_anti_resonance_bit, "enable_stall_detection_bit":self.driver.enable_stall_detection_bit, "use_backplane_proximity_bit":self.driver.use_backplane_proximity_bit, "use_encoder_bit":self.driver.use_encoder_bit, "home_to_encoder_z_pulse":self.driver.home_to_encoder_z_pulse, "input_3_function_bits":self.driver.input_3_function_bits, "input_2_function_bits":self.driver.input_2_function_bits, "input_1_function_bits":self.driver.input_1_function_bits, "output_functionality_bit":self.driver.output_functionality_bit, "output_state_control_on_network_lost":self.driver.output_state_control_on_network_lost, "output_state_on_network_lost":self.driver.output_state_on_network_lost, "read_present_configuration":self.driver.read_present_configuration, "save_configuration":self.driver.save_configuration, "binary_input_format":self.driver.binary_input_format, "binary_output_format":self.driver.binary_output_format, "binary_endian":self.driver.binary_endian, "input_3_active_level":self.driver.input_3_active_level, "input_2_active_level":self.driver.input_2_active_level, "input_1_active_level":self.driver.input_1_active_level, "starting_speed":self.driver.starting_speed, "motors_step_turn":self.driver.motors_step_turn, "hybrid_control_gain":self.driver.hybrid_control_gain, "encoder_pulses_turn":self.driver.encoder_pulses_turn, "idle_current_percentage":self.driver.idle_current_percentage, "motor_current":self.driver.motor_current, "current_loop_gain":self.driver.current_loop_gain}

        self.autohomeDlg = ConfigureAMCIDriverForm(parent=self, data=data)
        self.autohomeDlg.setWindowTitle("Configure Driver")
        if self.autohomeDlg.exec():
            self.driver.request_write_configuration(**data)

    def home_ccw(self):
        if self.ccw_finding_home:
            self.ccw_finding_home = False
            self.driver.request_write_return_to_command_mode()
            self.homeCCWButton.setEnabled(True)
            self.relativeMoveButton.setEnabled(True)
            self.presetEncoderButton.setEnabled(True)
            self.runAssemblyButton.setEnabled(True)
            self.prgAssemblyButton.setEnabled(True)
            self.resetErrorsButton.setEnabled(True)
            self.presetPositionButton.setEnabled(True)
            self.manualCWButton.setEnabled(True)
            self.homeCWButton.setEnabled(True)
            self.resumeMoveButton.setEnabled(True)
            self.manualCCWButton.setEnabled(True)
            self.absoluteMoveButton.setEnabled(not self.position_invalid)
        else:
            self.ccw_finding_home = True
            self.driver.request_write_ccw_find_home(programmed_speed=self.speed, acceleration=self.acceleration, deceleration=self.deceleration, acceleration_jerk=self.jerk)
            self.homeCCWButton.setEnabled(False)
            self.relativeMoveButton.setEnabled(False)
            self.presetEncoderButton.setEnabled(False)
            self.runAssemblyButton.setEnabled(False)
            self.prgAssemblyButton.setEnabled(False)
            self.resetErrorsButton.setEnabled(False)
            self.presetPositionButton.setEnabled(False)
            self.manualCWButton.setEnabled(False)
            self.homeCWButton.setEnabled(False)
            self.resumeMoveButton.setEnabled(False)
            self.manualCCWButton.setEnabled(False)
            self.absoluteMoveButton.setEnabled(False)

    def home_cw(self):
        if self.cw_finding_home:
            self.cw_finding_home = False
            self.driver.request_write_return_to_command_mode()
            self.homeCWButton.setEnabled(True)
            self.relativeMoveButton.setEnabled(True)
            self.presetEncoderButton.setEnabled(True)
            self.runAssemblyButton.setEnabled(True)
            self.prgAssemblyButton.setEnabled(True)
            self.resetErrorsButton.setEnabled(True)
            self.presetPositionButton.setEnabled(True)
            self.manualCWButton.setEnabled(True)
            self.homeCCWButton.setEnabled(True)
            self.resumeMoveButton.setEnabled(True)
            self.manualCCWButton.setEnabled(True)
            self.absoluteMoveButton.setEnabled(not self.position_invalid)
        else:
            self.cw_finding_home = True
            self.driver.request_write_cw_find_home(programmed_speed=self.speed, acceleration=self.acceleration, deceleration=self.deceleration, acceleration_jerk=self.jerk)
            self.homeCWButton.setEnabled(False)
            self.relativeMoveButton.setEnabled(False)
            self.presetEncoderButton.setEnabled(False)
            self.runAssemblyButton.setEnabled(False)
            self.prgAssemblyButton.setEnabled(False)
            self.resetErrorsButton.setEnabled(False)
            self.presetPositionButton.setEnabled(False)
            self.manualCWButton.setEnabled(False)
            self.homeCCWButton.setEnabled(False)
            self.resumeMoveButton.setEnabled(False)
            self.manualCCWButton.setEnabled(False)
            self.absoluteMoveButton.setEnabled(False)

    def immediate_stop(self):
        self.driver.request_write_immediate_stop()
        if self.absolute_moving:
            self.absolute_moving = False
        elif self.relative_moving:
            self.relative_moving = False
        elif self.manual_ccw_moving or self.manual_cw_moving:
            self.manual_ccw_moving = False
            self.manualCCWButton.setText('Manual CCW')
            self.manual_cw_moving = False
            self.manualCWButton.setText('Manual CW')
        elif self.running_assembly:
            self.running_assembly = False
        elif self.ccw_finding_home:
            self.ccw_finding_home = False
        elif self.cw_finding_home:
            self.cw_finding_home = False
        self.relativeMoveButton.setEnabled(True)
        self.presetEncoderButton.setEnabled(True)
        self.runAssemblyButton.setEnabled(True)
        self.prgAssemblyButton.setEnabled(True)
        self.resetErrorsButton.setEnabled(True)
        self.presetPositionButton.setEnabled(True)
        self.manualCWButton.setEnabled(True)
        self.homeCCWButton.setEnabled(True)
        self.homeCWButton.setEnabled(True)
        self.resumeMoveButton.setEnabled(True)
        self.manualCCWButton.setEnabled(True)
        self.absoluteMoveButton.setEnabled(not self.position_invalid)

    def hold_move(self):
        self.driver.request_write_hold_move()
        if self.absolute_moving:
            self.absolute_moving = False
            self.absolute_moving_on_hold = True
        elif self.relative_moving:
            self.relative_moving = False
            self.relative_moving_on_hold = True
        elif self.manual_ccw_moving:
            self.manual_ccw_moving = False
            self.manualCCWButton.setText('Manual CCW')
            self.manual_ccw_on_hold = True
        elif self.manual_cw_moving:
            self.manual_cw_moving = False
            self.manualCWButton.setText('Manual CW')
            self.manual_cw_on_hold = True
        elif self.running_assembly:
            self.running_assembly = False
        elif self.ccw_finding_home:
            self.ccw_finding_home = False
        elif self.cw_finding_home:
            self.cw_finding_home = False
        self.relativeMoveButton.setEnabled(True)
        self.presetEncoderButton.setEnabled(True)
        self.runAssemblyButton.setEnabled(True)
        self.prgAssemblyButton.setEnabled(True)
        self.resetErrorsButton.setEnabled(True)
        self.presetPositionButton.setEnabled(True)
        self.manualCWButton.setEnabled(True)
        self.homeCCWButton.setEnabled(True)
        self.homeCWButton.setEnabled(True)
        self.resumeMoveButton.setEnabled(True)
        self.manualCCWButton.setEnabled(True)
        self.absoluteMoveButton.setEnabled(not self.position_invalid)

    def resume(self):
        resumed = False
        if self.absolute_moving_on_hold:
            self.absolute_moving_on_hold = False
            self.absolute_moving = True
            self.manualCCWButton.setEnabled(False)
            self.manualCWButton.setEnabled(False)
            resumed = True
        elif self.relative_moving_on_hold:
            self.relative_moving_on_hold = False
            self.relative_moving = True
            self.manualCCWButton.setEnabled(False)
            self.manualCWButton.setEnabled(False)
            resumed = True
        elif self.manual_ccw_on_hold:
            self.manual_ccw_on_hold = False
            self.manual_ccw_moving = True
            self.manualCCWButton.setText('Stop')
            self.manualCWButton.setEnabled(False)
            resumed = True
        elif self.manual_cw_on_hold:
            self.manual_cw_on_hold = False
            self.manual_cw_moving = True
            self.manualCWButton.setText('Stop')
            self.manualCCWButton.setEnabled(False)
            resumed = True
        if resumed:
            self.driver.request_write_resume_move(self.speed, self.acceleration, self.deceleration, self.jerk)
            self.relativeMoveButton.setEnabled(False)
            self.absoluteMoveButton.setEnabled(False)
            self.presetEncoderButton.setEnabled(False)
            self.runAssemblyButton.setEnabled(False)
            self.prgAssemblyButton.setEnabled(False)
            self.resetErrorsButton.setEnabled(False)
            self.presetPositionButton.setEnabled(False)
            self.homeCCWButton.setEnabled(False)
            self.homeCWButton.setEnabled(False)
            self.resumeMoveButton.setEnabled(False)

    def relative_move(self):
        self.absolute_moving_on_hold = False
        self.relative_moving_on_hold = False
        self.manual_ccw_on_hold = False
        self.manual_cw_on_hold = False
        if self.relative_moving:
            self.relative_moving = False
            self.relativeMoveButton.setEnabled(True)
            self.presetEncoderButton.setEnabled(True)
            self.runAssemblyButton.setEnabled(True)
            self.prgAssemblyButton.setEnabled(True)
            self.resetErrorsButton.setEnabled(True)
            self.presetPositionButton.setEnabled(True)
            self.manualCWButton.setEnabled(True)
            self.homeCCWButton.setEnabled(True)
            self.homeCWButton.setEnabled(True)
            self.resumeMoveButton.setEnabled(True)
            self.manualCCWButton.setEnabled(True)
            self.absoluteMoveButton.setEnabled(not self.position_invalid)
            self.driver.request_write_return_to_command_mode()
        else:
            if self.position == 0:
                return
            self.relative_moving = True
            self.relativeMoveButton.setEnabled(False)
            self.presetEncoderButton.setEnabled(False)
            self.runAssemblyButton.setEnabled(False)
            self.prgAssemblyButton.setEnabled(False)
            self.resetErrorsButton.setEnabled(False)
            self.presetPositionButton.setEnabled(False)
            self.manualCWButton.setEnabled(False)
            self.homeCCWButton.setEnabled(False)
            self.homeCWButton.setEnabled(False)
            self.resumeMoveButton.setEnabled(False)
            self.manualCCWButton.setEnabled(False)
            self.absoluteMoveButton.setEnabled(False)
            self.driver.request_write_relative_move(self.position, programmed_speed=self.speed, acceleration=self.acceleration, deceleration=self.deceleration, acceleration_jerk=self.jerk)

    # def synchrostep_move(self):
    #     if self.synchrostep_moving:
    #         self.synchrostep_moving = False
    #         self.synchrostepMoveButton.setText('Synchrostep')
    #         self.absoluteMoveButton.setEnabled(not self.position_invalid)
    #         self.presetEncoderButton.setEnabled(True)
    #         self.runAssemblyButton.setEnabled(True)
    #         self.prgAssemblyButton.setEnabled(True)
    #         self.resetErrorsButton.setEnabled(True)
    #         self.presetPositionButton.setEnabled(True)
    #         self.manualCWButton.setEnabled(True)
    #         self.homeCCWButton.setEnabled(True)
    #         self.homeCWButton.setEnabled(True)
    #         self.resumeMoveButton.setEnabled(True)
    #         self.manualCCWButton.setEnabled(True)
    #         self.relativeMoveButton.setEnabled(True)
    #         self.driver.request_write_return_to_command_mode()
    #     else:
    #         self.signal_generator.start()
    #         self.synchrostep_moving = True
    #         self.synchrostepMoveButton.setText('Stop')
    #         self.absoluteMoveButton.setEnabled(False)
    #         self.presetEncoderButton.setEnabled(False)
    #         self.runAssemblyButton.setEnabled(False)
    #         self.prgAssemblyButton.setEnabled(False)
    #         self.resetErrorsButton.setEnabled(False)
    #         self.presetPositionButton.setEnabled(False)
    #         self.manualCWButton.setEnabled(False)
    #         self.homeCCWButton.setEnabled(False)
    #         self.homeCWButton.setEnabled(False)
    #         self.resumeMoveButton.setEnabled(False)
    #         self.manualCCWButton.setEnabled(False)
    #         self.relativeMoveButton.setEnabled(False)
    #         #self.driver.request_write_synchrostep_move(self.position, not self.reverse_blend_dir, speed=self.speed, acceleration=self.acceleration, deceleration=self.deceleration)

    def absolute_move(self):
        self.absolute_moving_on_hold = False
        self.relative_moving_on_hold = False
        self.manual_ccw_on_hold = False
        self.manual_cw_on_hold = False
        if self.absolute_moving:
            self.absolute_moving = False
            self.absoluteMoveButton.setEnabled(True)
            # self.synchrostepMoveButton.setEnabled(True)
            self.presetEncoderButton.setEnabled(True)
            self.runAssemblyButton.setEnabled(True)
            self.prgAssemblyButton.setEnabled(True)
            self.resetErrorsButton.setEnabled(True)
            self.presetPositionButton.setEnabled(True)
            self.manualCWButton.setEnabled(True)
            self.homeCCWButton.setEnabled(True)
            self.homeCWButton.setEnabled(True)
            self.resumeMoveButton.setEnabled(True)
            self.manualCCWButton.setEnabled(True)
            self.relativeMoveButton.setEnabled(True)
            self.driver.request_write_return_to_command_mode()
        else:
            if self.position == self.motor_position:
                return
            self.absolute_moving = True
            self.absoluteMoveButton.setEnabled(False)
            # self.synchrostepMoveButton.setEnabled(False)
            self.presetEncoderButton.setEnabled(False)
            self.runAssemblyButton.setEnabled(False)
            self.prgAssemblyButton.setEnabled(False)
            self.resetErrorsButton.setEnabled(False)
            self.presetPositionButton.setEnabled(False)
            self.manualCWButton.setEnabled(False)
            self.homeCCWButton.setEnabled(False)
            self.homeCWButton.setEnabled(False)
            self.resumeMoveButton.setEnabled(False)
            self.manualCCWButton.setEnabled(False)
            self.relativeMoveButton.setEnabled(False)
            self.driver.request_write_absolute_move(self.position, programmed_speed=self.speed, acceleration=self.acceleration, deceleration=self.deceleration, acceleration_jerk=self.jerk)

    def manual_ccw(self):
        self.absolute_moving_on_hold = False
        self.relative_moving_on_hold = False
        if self.manual_ccw_moving:
            self.manual_ccw_moving = False
            self.manualCCWButton.setText('Manual CCW')
            self.presetEncoderButton.setEnabled(True)
            self.runAssemblyButton.setEnabled(True)
            # self.synchrostepMoveButton.setEnabled(True)
            self.prgAssemblyButton.setEnabled(True)
            self.resetErrorsButton.setEnabled(True)
            self.presetPositionButton.setEnabled(True)
            self.manualCWButton.setEnabled(True)
            self.homeCCWButton.setEnabled(True)
            self.homeCWButton.setEnabled(True)
            self.resumeMoveButton.setEnabled(True)
            self.relativeMoveButton.setEnabled(True)
            self.absoluteMoveButton.setEnabled(not self.position_invalid)
            self.driver.request_write_return_to_command_mode()
        else:
            self.manual_ccw_moving = True
            self.manualCCWButton.setText('Stop')
            self.presetEncoderButton.setEnabled(False)
            self.runAssemblyButton.setEnabled(False)
            self.prgAssemblyButton.setEnabled(False)
            self.resetErrorsButton.setEnabled(False)
            # self.synchrostepMoveButton.setEnabled(False)
            self.presetPositionButton.setEnabled(False)
            self.manualCWButton.setEnabled(False)
            self.homeCCWButton.setEnabled(False)
            self.homeCWButton.setEnabled(False)
            self.resumeMoveButton.setEnabled(False)
            self.relativeMoveButton.setEnabled(False)
            self.absoluteMoveButton.setEnabled(False)
            self.driver.request_write_ccw_jog(programmed_speed=self.speed, acceleration=self.acceleration, deceleration=self.deceleration, acceleration_jerk=self.jerk)
    
    def manual_cw(self):
        self.absolute_moving_on_hold = False
        self.relative_moving_on_hold = False
        if self.manual_cw_moving:
            self.manual_cw_moving = False
            self.manualCWButton.setText('Manual CW')
            self.presetEncoderButton.setEnabled(True)
            # self.synchrostepMoveButton.setEnabled(True)
            self.runAssemblyButton.setEnabled(True)
            self.prgAssemblyButton.setEnabled(True)
            self.resetErrorsButton.setEnabled(True)
            self.presetPositionButton.setEnabled(True)
            self.manualCCWButton.setEnabled(True)
            self.homeCCWButton.setEnabled(True)
            self.homeCWButton.setEnabled(True)
            self.resumeMoveButton.setEnabled(True)
            self.relativeMoveButton.setEnabled(True)
            self.absoluteMoveButton.setEnabled(not self.position_invalid)
            self.driver.request_write_return_to_command_mode()
        else:
            self.manual_cw_moving = True
            self.manualCWButton.setText('Stop')
            self.presetEncoderButton.setEnabled(False)
            self.runAssemblyButton.setEnabled(False)
            self.prgAssemblyButton.setEnabled(False)
            # self.synchrostepMoveButton.setEnabled(False)
            self.resetErrorsButton.setEnabled(False)
            self.presetPositionButton.setEnabled(False)
            self.manualCCWButton.setEnabled(False)
            self.homeCCWButton.setEnabled(False)
            self.homeCWButton.setEnabled(False)
            self.resumeMoveButton.setEnabled(False)
            self.relativeMoveButton.setEnabled(False)
            self.absoluteMoveButton.setEnabled(False)
            self.driver.request_write_cw_jog(programmed_speed=self.speed, acceleration=self.acceleration, deceleration=self.deceleration, acceleration_jerk=self.jerk)

    def preset_position(self):
        self.driver.request_write_preset_position(self.position)

    def reset_errors(self):
        self.driver.request_write_reset_errors()

    def enable_disable_driver(self):
        if self.driver_is_enabled:
            self.driver_is_enabled = False
            self.disableDriverButton.setText('Enable Power')
            self.driver.request_write_disable_current()
        else:
            self.driver_is_enabled = True
            self.disableDriverButton.setText('Disable Power')
            self.driver.request_write_enable_current()

    def change_position(self, value):
        self.position = value

    def change_speed(self, value):
        self.speed = value
        if self.manual_ccw_moving:
            self.driver.request_write_ccw_jog(programmed_speed=self.speed, acceleration=self.acceleration, deceleration=self.deceleration, acceleration_jerk=self.jerk)
        if self.manual_cw_moving:
            self.driver.request_write_cw_jog(programmed_speed=self.speed, acceleration=self.acceleration, deceleration=self.deceleration, acceleration_jerk=self.jerk)

    def change_acceleration(self, value):
        self.acceleration = value

    def change_deceleration(self, value):
        self.deceleration = value

    def change_dwell_delay(self, value):
        self.dwell_delay = value

    def change_jerk(self, value):
        self.jerk = value

    def change_motor_position(self, value):
        self.motor_position = value
        self.currentPositionLabel.setText(str(value))
    
    def change_encoder_position(self, value):
        self.encoder_position = value
        self.encoderPositionLabel.setText(str(value))

    def change_captured_encoder_position(self, value):
        self.captured_encoder_position = value
        self.capturedEncoderPositionLabel.setText(str(value))

    def change_programed_motor_current(self, value):
        self.programed_motor_current = value
        self.motorCurrentLabel.setText(str(value))

    def change_acceleration_jerk(self, value):
        self.accelerating_jerk = value
        self.accelerationJerkLabel.setText(str(value))

    def switch_driver_is_enabled(self, value):
        self.driver_is_enabled_status.led.setChecked(value)
        if value != self.driver_is_enabled:
            if value:
                self.disableDriverButton.setText('Disable Power')
            else:
                self.disableDriverButton.setText('Enable Power')
            self.driver_is_enabled = value

    def switch_stall_detected(self, value):
        self.stall_detected_status.led.setChecked(value)

    def switch_output_state(self, value):
        self.output_state_status.led.setChecked(value)

    def switch_heartbeat_bit(self, value):
        self.heartbeat_bit_status.led.setChecked(value)

    def switch_limit_condition(self, value):
        self.limit_condition_status.led.setChecked(value)

    def switch_invalid_jog_change(self, value):
        self.invalid_jog_change_status.led.setChecked(value)

    def switch_motion_lag(self, value):
        self.motion_lag_status.led.setChecked(value)

    def switch_driver_fault(self, value):
        self.driver_fault_status.led.setChecked(value)

    def switch_connection_was_lost(self, value):
        self.connection_was_lost_status.led.setChecked(value)

    def switch_plc_in_prog_mode(self, value):
        self.plc_in_prog_mode_status.led.setChecked(value)

    def switch_temperature_above_90(self, value):
        self.temperature_above_90_status.led.setChecked(value)

    def switch_in_3_active(self, value):
        self.in_3_active_status.led.setChecked(value)

    def switch_in_2_active(self, value):
        #print(value)
        self.in_2_active_status.led.setChecked(value)

    def switch_in_1_active(self, value):
        self.in_1_active_status.led.setChecked(value)





    def switch_led_module_ok(self, value):
        self.module_ok_status.led.setChecked(value)    

    def switch_configuration_error(self, value):
        self.configuration_error_status.led.setChecked(value)

    def switch_command_error(self, value):
        self.command_error_status.led.setChecked(value)

    def switch_input_error(self, value):
        self.input_error_status.led.setChecked(value)

    def switch_position_invalid(self, value):
        self.position_invalid_status.led.setChecked(value)
        self.position_invalid = value
        self.absoluteMoveButton.setEnabled(not value)

    def switch_waiting_for_assembled_segment(self, value):
        self.waiting_for_assembled = value
        self.waiting_for_assembled_segment_status.led.setChecked(value)
        if self.programming_assembly:
            if value:
                if self.segment_assembly_index < self.assembly_data1['segments']:
                    self.driver.request_write_assembled_segment(target_position=self.assembly_data2[f'segment{self.segment_assembly_index+1}_pos'], programmed_speed=self.assembly_data2[f'segment{self.segment_assembly_index+1}_speed'], acceleration=self.assembly_data2[f'segment{self.segment_assembly_index+1}_acceleration'], deceleration=self.assembly_data2[f'segment{self.segment_assembly_index+1}_deceleration'], acceleration_jerk=self.assembly_data2[f'segment{self.segment_assembly_index+1}_jerk'])
                    self.segment_assembly_index += 1
                else:
                    self.programming_assembly = False
                    self.segment_assembly_index = 0
                    self.driver.request_write_return_to_command_mode()
            else:
                self.driver.request_write_program_assembled()

    def switch_in_assembled_mode(self, value):
        self.in_assembled_mode_status.led.setChecked(value)

    def switch_move_complete(self, value):
        self.move_complete_status.led.setChecked(value)
        if value and self.relative_moving:
            self.relative_move()
        if value and self.absolute_moving:
            self.absolute_move()
        if value and self.running_assembly:
            self.run_assembly()
        # if value and self.synchrostep_moving:
        #     self.synchrostep_move()

    def switch_decelerating(self, value):
        self.decelerating_status.led.setChecked(value)

    def switch_accelerating(self, value):
        self.accelerating_status.led.setChecked(value)

    def switch_at_home(self, value):
        self.at_home_status.led.setChecked(value)
        if value and self.ccw_finding_home:
            self.home_ccw()
        elif value and self.cw_finding_home:
            self.home_cw()

    def switch_stopped(self, value):
        self.stopped_status.led.setChecked(value)

    def switch_in_hold_state(self, value):
        self.in_hold_state_status.led.setChecked(value)

    def switch_moving_ccw(self, value):
        self.moving_ccw_status.led.setChecked(value)

    def switch_moving_cw(self, value):
        self.moving_cw_status.led.setChecked(value)

def main():
    z_event = threading.Event()
    z_event.set()
    z_driver = AMCIDriver('192.168.2.102', z_event, connected=True, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'])
    z_driver.start()

    app = QApplication(sys.argv)
    window = AMCIWidget(z_driver)
    window.show()
    app.exec_()


if __name__ == "__main__":
    main()