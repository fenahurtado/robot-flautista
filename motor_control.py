from tabnanny import verbose
import threading
from utils.driver_amci import AMCIDriver, INPUT_FUNCTION_BITS
from numpy import *
from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox, QFileDialog, QLineEdit, QPushButton, QVBoxLayout
    )
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import sys
from time import time, sleep
from random import random, randint
from functools import partial
from views.plot_window_2 import Ui_MainWindow as PlotWindow
import pandas as pd

signals = ['X', 'X_ref', 'Z', 'Z_ref', 'Alpha', 'Alpha_ref']

class Window(QMainWindow, PlotWindow):
    def __init__(self, app, measure, data, interval=10, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.interval = interval
        self.parent = parent
        self.app = app
        self.measures = {'X': False,
                         'X_ref': False,
                         'Z': False,
                         'Z_ref': False,
                         'Alpha': False,
                         'Alpha_ref': False}
        self.data = data
        self.actionX.triggered.connect(self.add_x_trace)
        self.actionX_ref.triggered.connect(self.add_x_ref_trace)
        self.actionZ.triggered.connect(self.add_z_trace)
        self.actionZ_ref.triggered.connect(self.add_z_ref_trace)
        self.actionAlpha.triggered.connect(self.add_alpha_trace)
        self.actionAlpha_ref.triggered.connect(self.add_alpha_ref_trace)

        colors = ['b', 'g', 'r', 'c', 'm', 'y']

        self.curves = [self.graphicsView.plot(pen=pg.mkPen(colors[0], width=1), name='X'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[1], width=1, style=QtCore.Qt.DashLine), name='X_ref'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[2], width=1), name='Z'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[3], width=1, style=QtCore.Qt.DashLine), name='Z_ref'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[4], width=1), name='Alpha'),
                       self.graphicsView.plot(pen=pg.mkPen(colors[5], width=1, style=QtCore.Qt.DashLine), name='Alpha_ref')]
        
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

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.interval)

        self.t0 = time()
        self.t = 0
    
    def add_x_trace(self, plot):
        self.measures['X'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[0], 'X')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[0])
            self.curves[0].setData([], [])

    def add_x_ref_trace(self, plot):
        self.measures['X_ref'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[1], 'X_ref')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[1])
            self.curves[1].setData([], [])
    
    def add_z_trace(self, plot):
        self.measures['Z'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[2], 'Z')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[2])
            self.curves[2].setData([], [])
    
    def add_z_ref_trace(self, plot):
        self.measures['Z_ref'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[3], 'Z_ref')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[3])
            self.curves[3].setData([], [])
    
    def add_alpha_trace(self, plot):
        self.measures['Alpha'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[4], 'Alpha')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[4])
            self.curves[4].setData([], [])
    
    def add_alpha_ref_trace(self, plot):
        self.measures['Alpha_ref'] = plot
        if plot:
            self.graphicsView.plotItem.legend.addItem(self.curves[5], 'Alpha_ref')
        else:
            self.graphicsView.plotItem.legend.removeItem(self.curves[5])
            self.curves[5].setData([], [])
            

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
            
        self.app.processEvents()

class Reference(QtCore.QThread):
    def __init__(self, driver, t0, route, move=False):
        QtCore.QThread.__init__(self)
        self.amp = 2000
        self.freq = 1
        self.t0 = t0
        self.ref = 0 #self.amp * sin(self.freq * (time() - self.t0))
        self.driver = driver
        self.move = move
        self.route = route

    def run(self):
        while True:
            sleep(0.1)
            self.ref = self.amp * sin(self.freq * (time() - self.t0))
            self.vel = self.amp * self.freq * cos(self.freq * (time() - self.t0))
            if self.move:
                #print(int(self.ref), int(self.vel))
                self.driver.request_write_synchrostep_move(int(self.ref), 0, speed=int(self.vel), acceleration=10, deceleration=10, proportional_coefficient=1, network_delay=10)
            # if self.mover and time() - self.t0 > 3:
            #     print('Moviendo motor...')
            #     self.driver.request_write_relative_move(2000)
            #     self.mover = False

class Recorder:
    def __init__(self, x_driver, z_driver, alpha_driver, x_reference, z_reference, alpha_reference, windowWidth=200, interval=10):
        self.saving = False
        self.x_driver = x_driver
        self.z_driver = z_driver
        self.alpha_driver = alpha_driver
        self.x_reference = x_reference
        self.z_reference = z_reference
        self.alpha_reference = alpha_reference
        self.windowWidth = windowWidth
        self.x_ref = linspace(0,0,self.windowWidth)
        self.z_ref = linspace(0,0,self.windowWidth)
        self.alpha_ref = linspace(0,0,self.windowWidth)
        self.x = linspace(0,0,self.windowWidth)
        self.z = linspace(0,0,self.windowWidth)
        self.alpha = linspace(0,0,self.windowWidth)
        self.times = linspace(0,0,self.windowWidth)
        self.t0 = time()

        self.data = pd.DataFrame(columns=['times', 'alpha', 'z', 'x', 'alpha_ref', 'z_ref', 'x_ref', 'flow_ref'])

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(interval)

    def update(self):
        self.x_ref[:-1] = self.x_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.x_ref[-1] = self.x_reference.ref
        self.z_ref[:-1] = self.z_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.z_ref[-1] = self.z_reference.ref
        self.alpha_ref[:-1] = self.alpha_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.alpha_ref[-1] = self.alpha_reference.ref
        self.x[:-1] = self.x[1:]                      # shift data in the temporal mean 1 sample left
        self.x[-1] = self.x_driver.motor_position
        self.z[:-1] = self.z[1:]                      # shift data in the temporal mean 1 sample left
        self.z[-1] = self.z_driver.motor_position
        self.alpha[:-1] = self.alpha[1:]                      # shift data in the temporal mean 1 sample left
        self.alpha[-1] = self.alpha_driver.motor_position
        self.times[:-1] = self.times[1:]                      # shift data in the temporal mean 1 sample left
        self.times[-1] = time() - self.t0

connected = True

x_event = threading.Event()
x_event.set()
x_driver = AMCIDriver('192.168.2.102', x_event, connected=connected, starting_speed=1, verbose=False)#, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'])
x_driver.start()

z_event = threading.Event()
z_event.set()
z_driver = AMCIDriver('192.168.2.104', z_event, connected=connected, starting_speed=1, verbose=False)#, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'])
z_driver.start()

alpha_event = threading.Event()
alpha_event.set()
alpha_driver = AMCIDriver('192.168.2.103', alpha_event, connected=connected, starting_speed=1, motors_step_turn=10000, verbose=False)
alpha_driver.start()

t0 = time()
x_reference = Reference(x_driver, t0, move=True)
x_reference.start()
z_reference = Reference(z_driver, t0, move=True)
z_reference.start()
alpha_reference = Reference(alpha_driver, t0, move=True)
alpha_reference.start()

x_driver.request_write_preset_position(0)
z_driver.request_write_preset_position(0)
alpha_driver.request_write_preset_position(0)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    measure = ['Z','Z_ref']
    recorder = Recorder(x_driver, z_driver, alpha_driver, x_reference, z_reference, alpha_reference)
    win = Window(app, measure, recorder)
    win.show()

    sys.exit(app.exec())