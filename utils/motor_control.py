import json
import os
from tabnanny import verbose
import threading
from utils.driver_amci import AMCIDriver, INPUT_FUNCTION_BITS
from utils.cinematica import *
from numpy import *
from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox, QFileDialog, QLineEdit, QPushButton, QVBoxLayout, QCheckBox, QWidgetAction, QLabel
    )
from PyQt5.QtCore import QUrl
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import sys
from time import time, sleep
from datetime import datetime
from random import random, randint
from functools import partial
from utils.driver_fingers import FingersDriver
from utils.sensores_alicat import FlowController, PreasureSensor
from views.plot_window_2 import Ui_MainWindow as PlotWindow
from view_control.forms import CalibrateAngleForm
import pandas as pd
from utils.motor_route import *

signals = ['X', 'X_ref', 'Z', 'Z_ref', 'Alpha', 'Alpha_ref']

class Window(QMainWindow, PlotWindow):
    def __init__(self, app, measure, player, interval=10, parent=None, independ=True):
        super().__init__(parent)
        self.setupUi(self)
        
        self.interval = interval
        self.parent = parent
        self.independ = independ
        self.app = app
        self.player = player
        self.x_driver = player.x_drive
        self.z_driver = player.z_drive
        self.alpha_driver = player.alpha_drive
        self.flow_controller = player.flow_controller
        self.preasure_sensor = player.preasure_sensor
        self.fingers_driver = player.fingers_driver
        self.x_ref = player.x_reference
        self.z_ref = player.z_reference
        self.alpha_ref = player.alpha_reference
        self.flow_ref = player.flow_reference
        self.fingers_ref = player.fingers_reference
        self.phrase_finished = False
        self.fingers_finished = False
        self.x_ref.finish_score_signal.connect(self.finish_score_phrase)
        self.fingers_ref.finish_score_signal.connect(self.finish_score_finger)
        #self.x_event = x_event
        #self.z_event = z_event
        #self.alpha_event = alpha_event
        #self.flow_event = flow_event
        #self.preasure_sensor_event = preasure_sensor_event
        #self.fingers_event = fingers_event
        #self.runEvent = runEvent
        #self.startEvent = startEvent
        self.base_path = os.path.dirname(os.path.realpath(__file__))

        self.measures = {'X': False,
                         'X_ref': False,
                         'Z': False,
                         'Z_ref': False,
                         'Alpha': False,
                         'Alpha_ref': False,
                         'Radio': False,
                         'Radio_ref': False,
                         'Theta': False,
                         'Theta_ref': False,
                         'Offset': False,
                         'Offset_ref': False,
                         'Flow': False,
                         'Flow_ref': False,
                         'Temperature': False,
                         'Preasure': False,
                         'Pitch': False}

        self.data = player.recorder
        self.route = None
        self.actionX.triggered.connect(self.add_x_trace)
        self.actionX_ref.triggered.connect(self.add_x_ref_trace)
        self.actionZ.triggered.connect(self.add_z_trace)
        self.actionZ_ref.triggered.connect(self.add_z_ref_trace)
        self.actionAlpha.triggered.connect(self.add_alpha_trace)
        self.actionAlpha_ref.triggered.connect(self.add_alpha_ref_trace)

        self.actionRadio.triggered.connect(self.add_radio_trace)
        self.actionRadio_ref.triggered.connect(self.add_radio_ref_trace)
        self.actionTheta.triggered.connect(self.add_theta_trace)
        self.actionTheta_ref.triggered.connect(self.add_theta_ref_trace)
        self.actionOffset.triggered.connect(self.add_offset_trace)
        self.actionOffset_ref.triggered.connect(self.add_offset_ref_trace)

        self.actionFlow.triggered.connect(self.add_flow_trace)
        self.actionFlow_ref.triggered.connect(self.add_flow_ref_trace)
        self.actionAir_Temperature.triggered.connect(self.add_air_temperature_trace)
        self.actionPreasure.triggered.connect(self.add_preasure_trace)
        self.actionPitch.triggered.connect(self.add_pitch_trace)

        #self.openScoreButton.clicked.connect(self.open_score)
        self.startButton.clicked.connect(self.start_playing)
        #self.stopButton.clicked.connect(self.stop_motors)
        self.x_driver.move_complete_signal.connect(self.x_move_complete)
        self.z_driver.move_complete_signal.connect(self.z_move_complete)
        self.alpha_driver.move_complete_signal.connect(self.alpha_move_complete)

        self.record_data = False
        self.recordDataCheckBox.toggled.connect(self.change_record_data)

        self.x_moving = False
        self.z_moving = False
        self.alpha_moving = False

        colors = ['b', 'g', 'r', 'c', 'm', 'y']

        self.curves = [self.graphicsView.plot(pen=pg.mkPen(colors[0], width=1), name='X'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[1], width=1, style=QtCore.Qt.DashLine), name='X_ref'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[2], width=1), name='Z'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[3], width=1, style=QtCore.Qt.DashLine), name='Z_ref'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[4], width=1), name='Alpha'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[5], width=1, style=QtCore.Qt.DashLine), name='Alpha_ref'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[0], width=1), name='Radio'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[1], width=1, style=QtCore.Qt.DashLine), name='Radio_ref'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[2], width=1), name='Theta'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[3], width=1, style=QtCore.Qt.DashLine), name='Theta_ref'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[4], width=1), name='Offset'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[5], width=1, style=QtCore.Qt.DashLine), name='Offset_ref'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[0], width=1), name='Flow'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[1], width=1, style=QtCore.Qt.DashLine), name='Flow_ref'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[2], width=1), name='Air Temperature'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[3], width=1), name='Preasure'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[4], width=1), name='Pitch')]
        
        self.graphicsView.addLegend()

        if 'X' in measure:
            self.measures['X'] = True
            self.actionX.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[0], 'X')
        if 'X_ref' in measure:
            self.measures['X_ref'] = True
            self.actionX_ref.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[1], 'X_ref')
        if 'Z' in measure:
            self.measures['Z'] = True
            self.actionZ.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[2], 'Z')
        if 'Z_ref' in measure:
            self.measures['Z_ref'] = True
            self.actionZ_ref.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[3], 'Z_ref')
        if 'Alpha' in measure:
            self.measures['Alpha'] = True
            self.actionAlpha.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[4], 'Alpha')
        if 'Alpha_ref' in measure:
            self.measures['Alpha_ref'] = True
            self.actionAlpha_ref.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[5], 'Alpha_ref')

        if 'Radio' in measure:
            self.measures['Radio'] = True
            self.actionRadio.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[6], 'Radio')
        if 'Radio_ref' in measure:
            self.measures['Radio_ref'] = True
            self.actionRadio_ref.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[7], 'Radio_ref')
        if 'Theta' in measure:
            self.measures['Theta'] = True
            self.actionTheta.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[8], 'Theta')
        if 'Theta_ref' in measure:
            self.measures['Theta_ref'] = True
            self.actionTheta_ref.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[9], 'Theta_ref')
        if 'Offset' in measure:
            self.measures['Offset'] = True
            self.actionOffset.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[10], 'Offset')
        if 'Offset_ref' in measure:
            self.measures['Offset_ref'] = True
            self.actionOffset_ref.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[11], 'Offset_ref')

        if 'Flow' in measure:
            self.measures['Flow'] = True
            self.actionFlow.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[12], 'Flow')
        if 'Flow_ref' in measure:
            self.measures['Flow_ref'] = True
            self.actionFlow_ref.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[13], 'Flow_ref')
        if 'Temperature' in measure:
            self.measures['Temperature'] = True
            self.actionAir_Temperature.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[14], 'Air_temperature')
        if 'Preasure' in measure:
            self.measures['Preasure'] = True
            self.actionPreasure.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[15], 'Preasure')
        if 'Pitch' in measure:
            self.measures['Pitch'] = True
            self.actionPitch.setChecked(True)
            self.graphicsView.plotItem.legend.addItem(self.curves[16], 'Pitch')

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.interval)

        self.t0 = time()
        self.t = 0

        if self.independ:
            self.autohome_routine()
    
    def change_record_data(self, value):
        self.record_data = value

    def finish_score_phrase(self):
        print("Phrase score completed")
        self.phrase_finished = True
        if self.fingers_finished:
            self.phrase_finished = False
            self.fingers_finished = False
            self.finish_score()

    def finish_score_finger(self):
        print("Finger score completed")
        self.fingers_finished = True
        if self.phrase_finished:
            self.phrase_finished = False
            self.fingers_finished = False
            self.finish_score()

    def autohome_routine(self):
        self.x_driver.request_write_ccw_find_home_to_limit()
        self.z_driver.request_write_ccw_find_home_to_limit()
        self.alpha_driver.request_write_preset_position(0)
        data=[0]
        #self.alpha_driver.request_write_reset_errors()
        self.autohomeDlg = CalibrateAngleForm(parent=self, data=data)
        self.autohomeDlg.angleSpinBox.valueChanged.connect(self.change_motor_angle)
        self.autohomeDlg.setWindowTitle("Choose parameters")
        if self.autohomeDlg.exec():
            pass
        while not self.x_driver.move_complete and not self.z_driver.move_complete and not self.alpha_driver.move_complete:
            sleep(0.1)
        self.x_driver.request_write_preset_position(0)
        self.z_driver.request_write_preset_position(0)
        self.alpha_driver.request_write_preset_position(0)
    
    def change_motor_angle(self, value):
        units = alpha_angle_to_units(value)
        self.alpha_driver.request_write_absolute_move(units, programmed_speed=10000, acceleration=5000, deceleration=5000)
        self.alpha_driver.request_write_return_to_command_mode()

    def closeEvent(self, a0: QtGui.QCloseEvent):
        self.parent.executeButton.setEnabled(True)
        self.player.startEvent.clear()
        # self.x_event.clear()
        # self.z_event.clear()
        # self.alpha_event.clear()
        # self.flow_event.clear()
        # self.preasure_sensor_event.clear()
        # self.fingers_event.clear()
        # self.startEvent.clear()
        # self.runEvent.clear()
        return super().closeEvent(a0)

    def stop_motors(self):
        if not self.x_driver.stopped:
            self.x_moving = False
            self.x_driver.request_write_hold_move()
        if not self.z_driver.stopped:
            self.x_moving = False
            self.z_driver.request_write_hold_move()
        if not self.alpha_driver.stopped:
            self.x_moving = False
            self.alpha_driver.request_write_hold_move()

    def x_move_complete(self, value):
        self.x_moving = not value
        #print(value, self.x_moving, self.z_moving, self.alpha_moving)
        if value and not self.z_moving and not self.alpha_moving:
            self.startButton.setEnabled(True)
        
    def z_move_complete(self, value):
        self.z_moving = not value
        #print(value, self.x_moving, self.z_moving, self.alpha_moving)
        if value and not self.x_moving and not self.alpha_moving:
            self.startButton.setEnabled(True)
    
    def alpha_move_complete(self, value):
        self.alpha_moving = not value
        #print(value, self.x_moving, self.z_moving, self.alpha_moving)
        if value and not self.x_moving and not self.z_moving:
            self.startButton.setEnabled(True)

    def add_x_trace(self, plot):
        self.measures['X'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[0], 'X')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[0])
            self.curves[0].setData([], [])
        self.menuEdit.show()

    def add_x_ref_trace(self, plot):
        self.measures['X_ref'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[1], 'X_ref')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[1])
            self.curves[1].setData([], [])
        self.menuEdit.show()

    def add_z_trace(self, plot):
        self.measures['Z'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[2], 'Z')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[2])
            self.curves[2].setData([], [])
        self.menuEdit.show()
    
    def add_z_ref_trace(self, plot):
        self.measures['Z_ref'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[3], 'Z_ref')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[3])
            self.curves[3].setData([], [])
        self.menuEdit.show()
    
    def add_alpha_trace(self, plot):
        self.measures['Alpha'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[4], 'Alpha')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[4])
            self.curves[4].setData([], [])
        self.menuEdit.show()
    
    def add_alpha_ref_trace(self, plot):
        self.measures['Alpha_ref'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[5], 'Alpha_ref')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[5])
            self.curves[5].setData([], [])
        self.menuEdit.show()
            
    def add_radio_trace(self, plot):
        self.measures['Radio'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[6], 'Radio')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[6])
            self.curves[6].setData([], [])
        self.menuEdit.show()

    def add_radio_ref_trace(self, plot):
        self.measures['Radio_ref'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[7], 'Radio_ref')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[7])
            self.curves[7].setData([], [])
        self.menuEdit.show()

    def add_theta_trace(self, plot):
        self.measures['Theta'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[8], 'Theta')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[8])
            self.curves[8].setData([], [])
        self.menuEdit.show()

    def add_theta_ref_trace(self, plot):
        self.measures['Theta_ref'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[9], 'Theta_ref')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[9])
            self.curves[9].setData([], [])
        self.menuEdit.show()
    
    def add_offset_trace(self, plot):
        self.measures['Offset'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[10], 'Offset')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[10])
            self.curves[10].setData([], [])
        self.menuEdit.show()

    def add_offset_ref_trace(self, plot):
        self.measures['Offset_ref'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[11], 'Offset_ref')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[11])
            self.curves[11].setData([], [])
        self.menuEdit.show()

    def add_flow_trace(self, plot):
        self.measures['Flow'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[12], 'Flow')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[12])
            self.curves[12].setData([], [])
        self.menuEdit.show()

    def add_flow_ref_trace(self, plot):
        self.measures['Flow_ref'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[13], 'Flow_ref')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[13])
            self.curves[13].setData([], [])
        self.menuEdit.show()
    
    def add_air_temperature_trace(self, plot):
        self.measures['Temperature'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[14], 'Air_temperature')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[14])
            self.curves[14].setData([], [])
        self.menuEdit.show()
    
    def add_preasure_trace(self, plot):
        self.measures['Preasure'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[15], 'Preasure')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[15])
            self.curves[15].setData([], [])
        self.menuEdit.show()
    
    def add_pitch_trace(self, plot):
        self.measures['Pitch'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[16], 'Preasure')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[16])
            self.curves[16].setData([], [])
        self.menuEdit.show()

    def update(self):
        if self.measures['X']:
            self.curves[0].setData(self.data.times, self.data.x)
        if self.measures['X_ref']:
            self.curves[1].setData(self.data.times, self.data.x_ref)
        if self.measures['Z']:
            self.curves[2].setData(self.data.times, self.data.z)
        if self.measures['Z_ref']:
            self.curves[3].setData(self.data.times, self.data.z_ref)
        if self.measures['Alpha']:
            self.curves[4].setData(self.data.times, self.data.alpha)
        if self.measures['Alpha_ref']:
            self.curves[5].setData(self.data.times, self.data.alpha_ref)

        if self.measures['Radio']:
            self.curves[6].setData(self.data.times, self.data.radius)
        if self.measures['Radio_ref']:
            self.curves[7].setData(self.data.times, self.data.radius_ref)
        if self.measures['Theta']:
            self.curves[8].setData(self.data.times, self.data.theta)
        if self.measures['Theta_ref']:
            self.curves[9].setData(self.data.times, self.data.theta_ref)
        if self.measures['Offset']:
            self.curves[10].setData(self.data.times, self.data.offset)
        if self.measures['Offset_ref']:
            self.curves[11].setData(self.data.times, self.data.offset_ref)

        if self.measures['Flow']:
            self.curves[12].setData(self.data.times, self.data.volume_flow)
        if self.measures['Flow_ref']:
            self.curves[13].setData(self.data.times, self.data.flow_ref)
        if self.measures['Temperature']:
            self.curves[14].setData(self.data.times, self.data.temperature)
        if self.measures['Preasure']:
            self.curves[15].setData(self.data.times, self.data.mouth_pressure)
        if self.measures['Pitch']:
            self.curves[16].setData(self.data.times, self.data.frequency)
            
        self.app.processEvents()

    def open_score(self, path=False):
        if not path:
            path, _ = QFileDialog.getOpenFileName(self, 'Open file', "/home/fernando/Dropbox","JSON files (*.json)")
        try:
            self.route = get_route_complete(path)

            x_init_dist = abs(self.x_driver.motor_position - self.route['x'][0])
            z_init_dist = abs(self.z_driver.motor_position - self.route['z'][0])
            alpha_init_dist = abs(self.alpha_driver.motor_position - self.route['alpha'][0])
            dist_max = max(x_init_dist, z_init_dist) #, alpha_init_dist)
            
            if dist_max != 0:
                x_init_vel = int(2000 * x_init_dist / dist_max)
                z_init_vel = int(2000 * z_init_dist / dist_max)
                alpha_init_vel = int(20000 * alpha_init_dist / dist_max)
            else:
                x_init_vel = 2000
                z_init_vel = 2000
                alpha_init_vel = 20000
            self.x_moving = False
            self.z_moving = False
            self.alpha_moving = False
            if self.x_driver.motor_position != self.route['x'][0]:
                self.x_driver.request_write_absolute_move(self.route['x'][0], programmed_speed=x_init_vel, acceleration=10, deceleration=10, motor_current=30, acceleration_jerk=0)
                self.x_moving = True
            if self.z_driver.motor_position != self.route['z'][0]:
                self.z_driver.request_write_absolute_move(self.route['z'][0], programmed_speed=z_init_vel, acceleration=10, deceleration=10, motor_current=30, acceleration_jerk=0)
                self.z_moving = True
            if self.alpha_driver.motor_position != self.route['alpha'][0]:
                self.alpha_driver.request_write_absolute_move(self.route['alpha'][0], programmed_speed=alpha_init_vel, acceleration=5000, deceleration=5000, motor_current=30, acceleration_jerk=0)
                self.alpha_moving = True
            if not self.x_moving and not self.z_moving and not self.alpha_moving:
                self.startButton.setEnabled(True)
            else: 
                self.startButton.setEnabled(False)

            self.x_ref.ref = self.route['x'][0]
            self.z_ref.ref = self.route['z'][0]
            self.alpha_ref.ref = self.route['alpha'][0]

            x = []
            x_vel = []
            z = []
            z_vel = []
            alpha = []
            alpha_vel = []
            flow = []
            
            for i in range(len(self.route['t'])):
                x.append((self.route['t'][i], self.route['x'][i]))
                x_vel.append((self.route['t'][i], self.route['x_vel'][i]))
                z.append((self.route['t'][i], self.route['z'][i]))
                z_vel.append((self.route['t'][i], self.route['z_vel'][i]))
                alpha.append((self.route['t'][i], self.route['alpha'][i]))
                alpha_vel.append((self.route['t'][i], self.route['alpha_vel'][i]))
                flow.append((self.route['t'][i], self.route['flow'][i]))

            self.x_ref.positions = x
            self.x_ref.velocities = x_vel
            self.z_ref.positions = z
            self.z_ref.velocities = z_vel
            self.alpha_ref.positions = alpha
            self.alpha_ref.velocities = alpha_vel
            self.flow_ref.flows = flow
            self.fingers_ref.notes = self.route['notes']
        except:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Couldn't open file.")
            msg.setInformativeText("File format does not coincide. Try with other file.")
            msg.setWindowTitle("File Error")
            #msg.setDetailedText("The details are as follows:")
            retval = msg.exec_()

    def start_playing(self):
        self.phrase_finished = False
        self.fingers_finished = False
        t0 = time()
        self.x_ref.t0 = t0
        self.z_ref.t0 = t0
        self.alpha_ref.t0 = t0
        self.flow_ref.t0 = t0
        self.fingers_ref.t0 = t0
        self.player.startEvent.set()
        if self.record_data:
            self.player.start_saving_data()
        self.startButton.setEnabled(False)
        self.recordDataCheckBox.setEnabled(False)
        #self.openScoreButton.setEnabled(False)
    
    def finish_score(self):
        self.player.startEvent.clear()
        if self.record_data:
            self.player.pause_saving_data()
            folder_path = self.base_path[:-5] + "data/"+datetime.now().strftime("%d-%m-%Y_%H:%M:%S")
            os.mkdir(folder_path)
            #path = "/home/fernando/Dropbox/UC/Magister/robot-flautista/data/" +  + "_"
            self.player.save_recorded_data(folder_path+"/data.csv", folder_path+"/audio.wav")
            # except:
            #     pass
        self.startButton.setEnabled(True)
        self.recordDataCheckBox.setEnabled(True)


class Reference(QtCore.QThread):
    finish_score_signal = QtCore.pyqtSignal()
    def __init__(self, runEvent, startEvent, driver, t0, acc=10, dec=10, proportional_coefficient=1, delay=0, move=False):
        QtCore.QThread.__init__(self)
        self.amp = 200
        self.freq = 1
        self.t0 = t0
        self.runEvent = runEvent
        self.startEvent = startEvent
        self.ref = 0 #self.amp * sin(self.freq * (time() - self.t0))
        self.vel = 0
        self.acc = acc
        self.dec = dec
        self.proportional_coefficient = proportional_coefficient
        self.delay = delay
        self.driver = driver
        self.move = move
        self.positions = []
        self.velocities = []

    def run(self):
        while self.runEvent.is_set():
            self.startEvent.wait(timeout=1)
            while self.startEvent.is_set(): 
                t = time() - self.t0
                if t - self.positions[-1][0] > 1:
                    self.finish_score_signal.emit()
                    break
                self.ref = get_value_from_func(t, self.positions)
                self.vel = get_value_from_func(t, self.velocities)
                self.driver.request_write_synchrostep_move(int(self.ref), 0, speed=int(self.vel), acceleration=self.acc, deceleration=self.dec, proportional_coefficient=self.proportional_coefficient, network_delay=self.delay)
                sleep(0.05)
            sleep(0.1)
            
            # if self.startEvent.is_set():
            #     self.ref = self.amp * sin(self.freq * (time() - self.t0))# + self.positions[0][1]
            #     self.vel = self.amp * self.freq * cos(self.freq * (time() - self.t0))
            #     self.driver.request_write_synchrostep_move(int(self.ref), 0, speed=int(self.vel), acceleration=10, deceleration=10, proportional_coefficient=1, network_delay=10)
            # sleep(0.1)

class FlowReference(QtCore.QThread):
    finish_score_signal = QtCore.pyqtSignal()
    def __init__(self, runEvent, startEvent, driver, t0, connected=True):
        QtCore.QThread.__init__(self)
        self.amp = 200
        self.t0 = t0
        self.runEvent = runEvent
        self.startEvent = startEvent
        self.ref = 0 #self.amp * sin(self.freq * (time() - self.t0))
        self.vel = 0
        self.driver = driver
        self.connected = connected
        self.flows = []

    def run(self):
        while self.runEvent.is_set():
            self.startEvent.wait(timeout=1)
            while self.startEvent.is_set(): 
                t = time() - self.t0
                if len(self.flows):
                    if t - self.flows[-1][0] > 1:
                        self.ref = 0
                        self.driver.change_ref(self.ref)
                        self.finish_score_signal.emit()
                        break
                self.ref = get_value_from_func(t, self.flows)
                self.driver.change_ref(self.ref)
                sleep(0.05)

class FingersReference(QtCore.QThread):
    finish_score_signal = QtCore.pyqtSignal()
    def __init__(self, runEvent, startEvent, driver, t0, connected=True):
        QtCore.QThread.__init__(self)
        self.t0 = t0
        self.runEvent = runEvent
        self.startEvent = startEvent
        self.ref = 'D3' #self.amp * sin(self.freq * (time() - self.t0))
        self.vel = 0
        self.driver = driver
        self.connected = connected
        self.notes = []

    def run(self):
        while self.runEvent.is_set():
            self.startEvent.wait(timeout=0.5)
            note_to_play = 0
            t_acumul = 0
            dt = 0.5
            while self.startEvent.is_set(): 
                t = time() - self.t0
                if t >= t_acumul and note_to_play < len(self.notes):
                    self.ref = self.notes[note_to_play][1]
                    # print(self.ref)
                    self.driver.request_finger_action(self.ref)

                    note_to_play += 1
                    if note_to_play < len(self.notes):
                        dt = self.notes[note_to_play][0] - t_acumul
                        t_acumul += dt
                    else:
                        self.finish_score_signal.emit()
                sleep(dt/10)
            sleep(0.5)


# class Recorder:
#     """
#     Esta clase se encarga de almacenar la historia de las variables medidas. windowWidth dice la cantidad de datos a almacenar e interval el tiempo (en milisegundos) para obtener una muestra.
#     """
#     def __init__(self, x_driver, z_driver, alpha_driver, flow_controller, preasure_sensor, x_reference, z_reference, alpha_reference, flow_reference, windowWidth=200, interval=10):
#         self.saving = False
#         self.x_driver = x_driver
#         self.z_driver = z_driver
#         self.alpha_driver = alpha_driver
#         self.flow_controller = flow_controller
#         self.preasure_sensor = preasure_sensor
#         self.x_reference = x_reference
#         self.z_reference = z_reference
#         self.alpha_reference = alpha_reference
#         self.flow_reference = flow_reference
#         self.windowWidth = windowWidth
#         self.ref_state = State(0,0,0,0)
#         self.real_state = State(0,0,0,0)

#         self.ref_state.x = x_units_to_mm(self.x_reference.ref)
#         self.ref_state.z = z_units_to_mm(self.z_reference.ref)
#         self.ref_state.alpha = alpha_units_to_angle(self.alpha_reference.ref)
#         self.ref_state.flow = self.flow_reference.ref
#         self.real_state.x = x_units_to_mm(self.x_driver.motor_position)
#         self.real_state.z = z_units_to_mm(self.z_driver.motor_position)
#         self.real_state.alpha = alpha_units_to_angle(self.alpha_driver.motor_position)
#         self.real_state.flow = self.flow_controller.values['vol_flow']

#         self.flow_ref = linspace(0,0,self.windowWidth)
#         self.x_ref = linspace(0,0,self.windowWidth)
#         self.z_ref = linspace(0,0,self.windowWidth)
#         self.alpha_ref = linspace(0,0,self.windowWidth)
#         self.x = linspace(0,0,self.windowWidth)
#         self.z = linspace(0,0,self.windowWidth)
#         self.alpha = linspace(0,0,self.windowWidth)
#         self.radius = linspace(0,0,self.windowWidth)
#         self.theta = linspace(0,0,self.windowWidth)
#         self.offset = linspace(0,0,self.windowWidth)
#         self.radius_ref = linspace(0,0,self.windowWidth)
#         self.theta_ref = linspace(0,0,self.windowWidth)
#         self.offset_ref = linspace(0,0,self.windowWidth)
#         self.mouth_pressure = linspace(0,0,self.windowWidth)
#         self.volume_flow = linspace(0,0,self.windowWidth)
#         self.mass_flow = linspace(0,0,self.windowWidth)
#         self.temperature = linspace(0,0,self.windowWidth)
#         self.frequency = linspace(0,0,self.windowWidth)
#         self.times = linspace(0,0,self.windowWidth)
#         self.t0 = time()

#         self.data = pd.DataFrame(columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'offset_ref', 'theta_ref', 'radius_ref', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'])

#         self.interval = interval
#         self.timer = QtCore.QTimer()
#         self.timer.timeout.connect(self.update)
#         self.timer.start(interval)
        
#     def start(self):
#         self.timer.start(self.interval)

#     def update(self):
#         #self.flow_ref[:-1] = self.flow_ref[1:]                      # shift data in the temporal mean 1 sample left
#         #self.flow_ref[-1] = self.flowController.values['set_point']
        
#         self.ref_state.x = x_units_to_mm(self.x_reference.ref)
#         self.ref_state.z = z_units_to_mm(self.z_reference.ref)
#         self.ref_state.alpha = alpha_units_to_angle(self.alpha_reference.ref)
#         self.ref_state.flow = self.flow_reference.ref
#         self.real_state.x = x_units_to_mm(self.x_driver.motor_position)
#         self.real_state.z = z_units_to_mm(self.z_driver.motor_position)
#         self.real_state.alpha = alpha_units_to_angle(self.alpha_driver.motor_position)
#         self.real_state.flow = self.flow_controller.values['vol_flow']

#         self.x_ref[:-1] = self.x_ref[1:]                      # shift data in the temporal mean 1 sample left
#         self.x_ref[-1] = self.ref_state.x
#         self.z_ref[:-1] = self.z_ref[1:]                      # shift data in the temporal mean 1 sample left
#         self.z_ref[-1] = self.ref_state.z
#         self.alpha_ref[:-1] = self.alpha_ref[1:]                      # shift data in the temporal mean 1 sample left
#         self.alpha_ref[-1] = self.ref_state.alpha
#         self.flow_ref[:-1] = self.flow_ref[1:]                      # shift data in the temporal mean 1 sample left
#         self.flow_ref[-1] = self.ref_state.flow
#         self.x[:-1] = self.x[1:]                      # shift data in the temporal mean 1 sample left
#         self.x[-1] = self.real_state.x
#         self.z[:-1] = self.z[1:]                      # shift data in the temporal mean 1 sample left
#         self.z[-1] = self.real_state.z
#         self.alpha[:-1] = self.alpha[1:]                      # shift data in the temporal mean 1 sample left
#         self.alpha[-1] = self.real_state.alpha
#         self.volume_flow[:-1] = self.volume_flow[1:]                      # shift data in the temporal mean 1 sample left
#         self.volume_flow[-1] = self.real_state.flow

#         self.radius[:-1] = self.radius[1:]                      # shift data in the temporal mean 1 sample left
#         self.radius[-1] = self.real_state.r
#         self.theta[:-1] = self.theta[1:]                      # shift data in the temporal mean 1 sample left
#         self.theta[-1] = self.real_state.theta
#         self.offset[:-1] = self.offset[1:]                      # shift data in the temporal mean 1 sample left
#         self.offset[-1] = self.real_state.o
#         self.radius_ref[:-1] = self.radius_ref[1:]                      # shift data in the temporal mean 1 sample left
#         self.radius_ref[-1] = self.ref_state.r
#         self.theta_ref[:-1] = self.theta_ref[1:]                      # shift data in the temporal mean 1 sample left
#         self.theta_ref[-1] = self.ref_state.theta
#         self.offset_ref[:-1] = self.offset_ref[1:]                      # shift data in the temporal mean 1 sample left
#         self.offset_ref[-1] = self.ref_state.o
#         self.mouth_pressure[:-1] = self.mouth_pressure[1:]                      # shift data in the temporal mean 1 sample left
#         self.mouth_pressure[-1] = self.preasure_sensor.values['pressure']
#         #self.volume_flow[:-1] = self.volume_flow[1:]                      # shift data in the temporal mean 1 sample left
#         #self.volume_flow[-1] = self.flowController.values['vol_flow']
#         #self.mass_flow[:-1] = self.mass_flow[1:]                      # shift data in the temporal mean 1 sample left
#         #self.mass_flow[-1] = self.flowController.values['mass_flow']
#         self.temperature[:-1] = self.temperature[1:]                      # shift data in the temporal mean 1 sample left
#         self.temperature[-1] = self.flow_controller.values['temperature']
#         #self.frequency[:-1] = self.frequency[1:]                      # shift data in the temporal mean 1 sample left
#         #self.frequency[-1] = self.microphone.pitch
#         self.times[:-1] = self.times[1:]                      # shift data in the temporal mean 1 sample left
#         self.times[-1] = time() - self.t0
#         if self.saving:
#             new_data = pd.DataFrame([[self.times[-1], self.frequency[-1], self.temperature[-1], self.mass_flow[-1], self.volume_flow[-1], self.mouth_pressure[-1], self.offset[-1], self.theta[-1], self.radius[-1], self.offset_ref[-1], self.theta_ref[-1], self.radius_ref[-1], self.alpha[-1], self.z[-1], self.x[-1], self.alpha_ref[-1], self.z_ref[-1], self.x_ref[-1], self.alpha_ref[-1]]], columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'offset_ref', 'theta_ref', 'radius_ref', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'])
#             self.data = pd.concat([self.data, new_data], ignore_index=True)
    
#     def start_saving(self):
#         self.data = pd.DataFrame(columns=['times','frequency','temperature','mass_flow', 'volume_flow', 'mouth_pressure', 'offset', 'theta', 'radius', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'])
#         self.saving = True
    
#     def pause_saving(self):
#         self.saving = False

#     def resume_saving(self):
#         self.saving = True
    
#     def finish_saving(self, file_name):
#         self.saving = False
#         self.data.to_csv(file_name)

# class Recorder:
#     def __init__(self, x_driver, z_driver, alpha_driver, x_reference, z_reference, alpha_reference, windowWidth=200, interval=10):
#         self.saving = False
#         self.x_driver = x_driver
#         self.z_driver = z_driver
#         self.alpha_driver = alpha_driver
#         self.x_reference = x_reference
#         self.z_reference = z_reference
#         self.alpha_reference = alpha_reference
#         self.windowWidth = windowWidth
#         self.x_ref = linspace(0,0,self.windowWidth)
#         self.z_ref = linspace(0,0,self.windowWidth)
#         self.alpha_ref = linspace(0,0,self.windowWidth)
#         self.x = linspace(0,0,self.windowWidth)
#         self.z = linspace(0,0,self.windowWidth)
#         self.alpha = linspace(0,0,self.windowWidth)
#         self.times = linspace(0,0,self.windowWidth)
#         self.t0 = time()

#         self.data = pd.DataFrame(columns=['times', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'])

#         self.timer = QtCore.QTimer()
#         self.timer.timeout.connect(self.update)
#         self.timer.start(interval)

#     def update(self):
#         self.x_ref[:-1] = self.x_ref[1:]                      # shift data in the temporal mean 1 sample left
#         self.x_ref[-1] = self.x_reference.ref
#         self.z_ref[:-1] = self.z_ref[1:]                      # shift data in the temporal mean 1 sample left
#         self.z_ref[-1] = self.z_reference.ref
#         self.alpha_ref[:-1] = self.alpha_ref[1:]                      # shift data in the temporal mean 1 sample left
#         self.alpha_ref[-1] = self.alpha_reference.ref
#         self.x[:-1] = self.x[1:]                      # shift data in the temporal mean 1 sample left
#         self.x[-1] = self.x_driver.motor_position
#         self.z[:-1] = self.z[1:]                      # shift data in the temporal mean 1 sample left
#         self.z[-1] = self.z_driver.motor_position
#         self.alpha[:-1] = self.alpha[1:]                      # shift data in the temporal mean 1 sample left
#         self.alpha[-1] = self.alpha_driver.motor_position
#         self.times[:-1] = self.times[1:]                      # shift data in the temporal mean 1 sample left
#         self.times[-1] = time() - self.t0

if __name__ == '__main__':
    connected = True

    x_event = threading.Event()
    x_event.set()
    x_driver = AMCIDriver('192.168.2.102', x_event, connected=connected, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'])#, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'])
    x_driver.start()

    z_event = threading.Event()
    z_event.set()
    z_driver = AMCIDriver('192.168.2.104', z_event, connected=connected, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'])#, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'])
    z_driver.start()

    alpha_event = threading.Event()
    alpha_event.set()
    alpha_driver = AMCIDriver('192.168.2.103', alpha_event, connected=connected, starting_speed=1, motors_step_turn=10000, verbose=False)
    alpha_driver.start()

    flow_controler_event = threading.Event()
    flow_controler_event.set()
    flow_controller = FlowController('192.168.2.101', flow_controler_event, connected=connected)
    flow_controller.start()

    preasure_sensor_event = threading.Event()
    preasure_sensor_event.set()
    preasure_sensor = PreasureSensor('192.168.2.100', preasure_sensor_event, connected=connected)
    preasure_sensor.start()

    fingers_event = threading.Event()
    fingers_event.set()
    ## TEFO: '/dev/cu.usbserial-142420'
    fingers_driver = FingersDriver('/dev/ttyUSB0', fingers_event, connected=connected)
    fingers_driver.start()

    t0 = time()
    runEvent = threading.Event()
    runEvent.set()
    startEvent = threading.Event()
    x_reference = Reference(runEvent, startEvent, x_driver, t0, move=True, delay=20)
    x_reference.start()
    z_reference = Reference(runEvent, startEvent, z_driver, t0, move=True, delay=20)
    z_reference.start()
    alpha_reference = Reference(runEvent, startEvent, alpha_driver, t0, acc=5000, dec=5000, proportional_coefficient=10, delay=20, move=True)
    alpha_reference.start()
    flow_reference = FlowReference(runEvent, startEvent, flow_controller, t0, connected=True)
    flow_reference.start()
    fingers_reference = FingersReference(runEvent, startEvent, fingers_driver, t0, connected=True)
    fingers_reference.start()

    #x_driver.request_write_preset_position(0)
    #z_driver.request_write_preset_position(0)
    #alpha_driver.request_write_preset_position(0)

    app = QApplication(sys.argv)
    measure = ['X','X_ref']
    recorder = Recorder(x_driver, z_driver, alpha_driver, flow_controller, preasure_sensor, x_reference, z_reference, alpha_reference, flow_reference)
    win = Window(app, measure, recorder, x_driver, z_driver, alpha_driver, flow_controller, preasure_sensor, fingers_driver, x_reference, z_reference, alpha_reference, flow_reference, fingers_reference, x_event, z_event, alpha_event, flow_controler_event, preasure_sensor_event, fingers_event, runEvent, startEvent)
    win.show()

    sys.exit(app.exec())