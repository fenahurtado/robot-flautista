import sys
sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtCore
import pyqtgraph as pg

from utils.cinematica import *
from utils.motor_route import *
from utils.motor_control import Reference
from view_control.plot_window import Window as PlotWindow
from utils.driver_amci import AMCIDriver, INPUT_FUNCTION_BITS
from views.simple_plot_window import Ui_MainWindow as PlotWindowView

import matplotlib.pyplot as plt
from time import time
from datetime import datetime
import pandas as pd

class Window(QMainWindow, PlotWindowView):
    def __init__(self, app, data, x_reference, start_event, interval=10, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.interval = interval
        self.parent = parent
        self.x_reference = x_reference
        self.start_event = start_event
        self.app = app
        self.data = data
        self.curves = [self.graphicsView.plot(pen=pg.mkPen('b', width=1)), 
                       self.graphicsView.plot(pen=pg.mkPen('r', width=1, style=QtCore.Qt.DashLine))]
        
        self.x_reference.finish_score_signal.connect(self.stop_action)
        self.startButton.clicked.connect(self.start_action)
        self.stopButton.clicked.connect(self.stop_action)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.interval)

        self.t0 = time()
        self.t = 0
    
    def start_action(self):
        self.x_reference.t0 = time()
        self.data.start_saving()
        self.start_event.set()

    def stop_action(self):
        if self.start_event.is_set():
            filename = "/home/fernando/Dropbox/UC/Magister/robot-flautista/exercises/data/" + datetime.now().strftime("%d-%m-%Y_%H:%M:%S") + ".csv"
            self.data.finish_saving(filename)
        self.start_event.clear()
        

    def update(self):
        self.curves[0].setData(self.data.times, self.data.x)
        self.curves[1].setData(self.data.times, self.data.x_ref)
        self.app.processEvents()

class Window2(QMainWindow, PlotWindowView):
    def __init__(self, app, data, x_reference, z_reference, start_event, interval=10, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.interval = interval
        self.parent = parent
        self.x_reference = x_reference
        self.z_reference = z_reference
        self.start_event = start_event
        self.app = app
        self.data = data
        self.curves = [self.graphicsView.plot(pen=pg.mkPen('b', width=1)), 
                       self.graphicsView.plot(pen=pg.mkPen('r', width=1, style=QtCore.Qt.DashLine)),
                       self.graphicsView.plot(pen=pg.mkPen('g', width=1)),
                       self.graphicsView.plot(pen=pg.mkPen('w', width=1, style=QtCore.Qt.DashLine))]
        
        self.x_reference.finish_score_signal.connect(self.stop_action)
        self.startButton.clicked.connect(self.start_action)
        self.stopButton.clicked.connect(self.stop_action)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.interval)

        self.t0 = time()
        self.t = 0
    
    def start_action(self):
        t = time()
        self.x_reference.t0 = t
        self.z_reference.t0 = t
        self.data.start_saving()
        self.start_event.set()

    def stop_action(self):
        if self.start_event.is_set():
            filename = "/home/fernando/Dropbox/UC/Magister/robot-flautista/exercises/data/" + datetime.now().strftime("%d-%m-%Y_%H:%M:%S") + ".csv"
            self.data.finish_saving(filename)
        self.start_event.clear()
        

    def update(self):
        self.curves[0].setData(self.data.times, self.data.x)
        self.curves[1].setData(self.data.times, self.data.x_ref)
        self.curves[2].setData(self.data.times, self.data.z)
        
        self.app.processEvents()

class Window3(QMainWindow, PlotWindowView):
    def __init__(self, app, data, x_reference, z_reference, alpha_reference, start_event, interval=10, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.interval = interval
        self.parent = parent
        self.x_reference = x_reference
        self.z_reference = z_reference
        self.alpha_reference = alpha_reference
        self.start_event = start_event
        self.app = app
        self.data = data
        self.curves = [self.graphicsView.plot(pen=pg.mkPen('b', width=1)), 
                       self.graphicsView.plot(pen=pg.mkPen('r', width=1, style=QtCore.Qt.DashLine)),
                       self.graphicsView.plot(pen=pg.mkPen('g', width=1)),
                       self.graphicsView.plot(pen=pg.mkPen('w', width=1, style=QtCore.Qt.DashLine)),
                       self.graphicsView.plot(pen=pg.mkPen('c', width=1)),
                       self.graphicsView.plot(pen=pg.mkPen('y', width=1, style=QtCore.Qt.DashLine))]
        
        self.x_reference.finish_score_signal.connect(self.stop_action)
        self.startButton.clicked.connect(self.start_action)
        self.stopButton.clicked.connect(self.stop_action)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.interval)

        self.t0 = time()
        self.t = 0
    
    def start_action(self):
        t = time()
        self.x_reference.t0 = t
        self.z_reference.t0 = t
        self.alpha_reference.t0 = t
        self.data.start_saving()
        self.start_event.set()

    def stop_action(self):
        if self.start_event.is_set():
            filename = "/home/fernando/Dropbox/UC/Magister/robot-flautista/exercises/data/" + datetime.now().strftime("%d-%m-%Y_%H:%M:%S") + ".csv"
            self.data.finish_saving(filename)
        self.start_event.clear()
        

    def update(self):
        self.curves[0].setData(self.data.times, self.data.x)
        self.curves[1].setData(self.data.times, self.data.x_ref)
        #self.curves[2].setData(self.data.times, self.data.z)
        # self.curves[3].setData(self.data.times, self.data.z_ref)
        # self.curves[4].setData(self.data.times, self.data.alpha)
        # self.curves[5].setData(self.data.times, self.data.alpha_ref)

        self.app.processEvents()

class SimpleRecorder:
    """
    Esta clase se encarga de almacenar la historia de las variables medidas. windowWidth dice la cantidad de datos a almacenar e interval el tiempo (en milisegundos) para obtener una muestra.
    """
    def __init__(self, x_driver, x_reference, windowWidth=200, interval=10):
        self.saving = False
        self.x_driver = x_driver
        self.x_reference = x_reference
        self.windowWidth = windowWidth
        self.ref_state = 0
        self.real_state = 0

        self.ref_state = x_units_to_mm(self.x_reference.ref)
        self.real_state = x_units_to_mm(self.x_driver.motor_position)

        self.x_ref = linspace(0,0,self.windowWidth)
        self.x = linspace(0,0,self.windowWidth)
        self.times = linspace(0,0,self.windowWidth)

        self.t0 = time()
        self.first_entry = False
        self.t1 = 0

        self.data = pd.DataFrame(columns=['time', 'signal', 'value'])

        self.interval = interval
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(interval)

    def start(self):
        self.timer.start(self.interval)

    def update(self):
        self.ref_state = x_units_to_mm(self.x_reference.ref)
        self.real_state = x_units_to_mm(self.x_driver.motor_position)

        self.x_ref[:-1] = self.x_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.x_ref[-1] = self.ref_state
        self.x[:-1] = self.x[1:]                      # shift data in the temporal mean 1 sample left
        self.x[-1] = self.real_state

        self.times[:-1] = self.times[1:]                      # shift data in the temporal mean 1 sample left
        self.times[-1] = time() - self.t0

        if self.saving:
            if self.first_entry:
                self.t1 = self.times[-1]
                self.first_entry = False
            t = self.times[-1] - self.t1
            new_data = pd.DataFrame({'time': [t, t], 'signal':['x_ref', 'x'], 'value': [self.x_ref[-1], self.x[-1]]})
            self.data = self.data.append(new_data, ignore_index = True)
            
    def start_saving(self):
        self.first_entry = True
        self.data = pd.DataFrame(columns=['time', 'signal', 'value'])
        self.saving = True
    
    def pause_saving(self):
        self.saving = False

    def resume_saving(self):
        self.saving = True
    
    def finish_saving(self, file_name):
        self.saving = False
        self.data.to_csv(file_name)

class SimpleRecorder2:
    """
    Esta clase se encarga de almacenar la historia de las variables medidas. windowWidth dice la cantidad de datos a almacenar e interval el tiempo (en milisegundos) para obtener una muestra.
    """
    def __init__(self, x_driver, z_driver, x_reference, z_reference, windowWidth=200, interval=10):
        self.saving = False
        self.x_driver = x_driver
        self.z_driver = z_driver
        self.x_reference = x_reference
        self.z_reference = z_reference
        self.windowWidth = windowWidth
        self.ref_state = 0
        self.real_state = 0

        self.ref_state_x = x_units_to_mm(self.x_reference.ref)
        self.real_state_x = x_units_to_mm(self.x_driver.motor_position)
        self.ref_state_z = z_units_to_mm(self.z_reference.ref)
        self.real_state_z = z_units_to_mm(self.z_driver.motor_position)

        self.x_ref = linspace(0,0,self.windowWidth)
        self.x = linspace(0,0,self.windowWidth)
        self.z_ref = linspace(0,0,self.windowWidth)
        self.z = linspace(0,0,self.windowWidth)

        self.times = linspace(0,0,self.windowWidth)

        self.t0 = time()
        self.first_entry = False
        self.t1 = 0

        self.data = pd.DataFrame(columns=['time', 'signal', 'value'])

        self.interval = interval
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(interval)

    def start(self):
        self.timer.start(self.interval)

    def update(self):
        self.ref_state_x = x_units_to_mm(self.x_reference.ref)
        self.real_state_x = x_units_to_mm(self.x_driver.motor_position)
        self.ref_state_z = z_units_to_mm(self.z_reference.ref)
        self.real_state_z = z_units_to_mm(self.z_driver.motor_position)

        self.x_ref[:-1] = self.x_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.x_ref[-1] = self.ref_state_x
        self.x[:-1] = self.x[1:]                      # shift data in the temporal mean 1 sample left
        self.x[-1] = self.real_state_x

        self.z_ref[:-1] = self.z_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.z_ref[-1] = self.ref_state_z
        self.z[:-1] = self.z[1:]                      # shift data in the temporal mean 1 sample left
        self.z[-1] = self.real_state_z

        self.times[:-1] = self.times[1:]                      # shift data in the temporal mean 1 sample left
        self.times[-1] = time() - self.t0

        if self.saving:
            if self.first_entry:
                self.t1 = self.times[-1]
                self.first_entry = False
            t = self.times[-1] - self.t1
            new_data = pd.DataFrame({'time': [t, t, t, t], 'signal':['x_ref', 'x', 'z_ref', 'z'], 'value': [self.x_ref[-1], self.x[-1], self.z_ref[-1], self.z[-1]]})
            self.data = self.data.append(new_data, ignore_index = True)
            
    def start_saving(self):
        self.first_entry = True
        self.data = pd.DataFrame(columns=['time', 'signal', 'value'])
        self.saving = True
    
    def pause_saving(self):
        self.saving = False

    def resume_saving(self):
        self.saving = True
    
    def finish_saving(self, file_name):
        self.saving = False
        self.data.to_csv(file_name)

class SimpleRecorder3:
    """
    Esta clase se encarga de almacenar la historia de las variables medidas. windowWidth dice la cantidad de datos a almacenar e interval el tiempo (en milisegundos) para obtener una muestra.
    """
    def __init__(self, x_driver, z_driver, alpha_driver, x_reference, z_reference, alpha_reference, windowWidth=200, interval=10):
        self.saving = False
        self.x_driver = x_driver
        self.z_driver = z_driver
        self.alpha_driver = alpha_driver
        self.x_reference = x_reference
        self.z_reference = z_reference
        self.alpha_reference = alpha_reference
        self.windowWidth = windowWidth
        self.ref_state = 0
        self.real_state = 0

        self.ref_state_x = x_units_to_mm(self.x_reference.ref)
        self.real_state_x = x_units_to_mm(self.x_driver.motor_position)
        self.ref_state_z = z_units_to_mm(self.z_reference.ref)
        self.real_state_z = z_units_to_mm(self.z_driver.motor_position)
        self.ref_state_alpha = alpha_units_to_angle(self.alpha_reference.ref)
        self.real_state_alpha = alpha_units_to_angle(self.alpha_driver.motor_position)

        self.x_ref = linspace(0,0,self.windowWidth)
        self.x = linspace(0,0,self.windowWidth)
        self.z_ref = linspace(0,0,self.windowWidth)
        self.z = linspace(0,0,self.windowWidth)
        self.alpha_ref = linspace(0,0,self.windowWidth)
        self.alpha = linspace(0,0,self.windowWidth)

        self.times = linspace(0,0,self.windowWidth)

        self.t0 = time()
        self.first_entry = False
        self.t1 = 0

        self.data = pd.DataFrame(columns=['time', 'signal', 'value'])

        self.interval = interval
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(interval)

    def start(self):
        self.timer.start(self.interval)

    def update(self):
        self.ref_state_x = x_units_to_mm(self.x_reference.ref)
        self.real_state_x = x_units_to_mm(self.x_driver.motor_position)
        self.ref_state_z = z_units_to_mm(self.z_reference.ref)
        self.real_state_z = z_units_to_mm(self.z_driver.motor_position)
        self.ref_state_alpha = alpha_units_to_angle(self.alpha_reference.ref)
        self.real_state_alpha = alpha_units_to_angle(self.alpha_driver.motor_position)

        self.x_ref[:-1] = self.x_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.x_ref[-1] = self.ref_state_x
        self.x[:-1] = self.x[1:]                      # shift data in the temporal mean 1 sample left
        self.x[-1] = self.real_state_x

        self.z_ref[:-1] = self.z_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.z_ref[-1] = self.ref_state_z
        self.z[:-1] = self.z[1:]                      # shift data in the temporal mean 1 sample left
        self.z[-1] = self.real_state_z

        self.alpha_ref[:-1] = self.alpha_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.alpha_ref[-1] = self.ref_state_alpha
        self.alpha[:-1] = self.alpha[1:]                      # shift data in the temporal mean 1 sample left
        self.alpha[-1] = self.real_state_alpha

        self.times[:-1] = self.times[1:]                      # shift data in the temporal mean 1 sample left
        self.times[-1] = time() - self.t0

        if self.saving:
            if self.first_entry:
                self.t1 = self.times[-1]
                self.first_entry = False
            t = self.times[-1] - self.t1
            new_data = pd.DataFrame({'time': [t, t, t, t, t, t], 'signal':['x_ref', 'x', 'z_ref', 'z', 'alpha_ref', 'alpha'], 'value': [self.x_ref[-1], self.x[-1], self.z_ref[-1], self.z[-1], self.alpha_ref[-1], self.alpha[-1]]})
            self.data = self.data.append(new_data, ignore_index = True)
            
    def start_saving(self):
        self.first_entry = True
        self.data = pd.DataFrame(columns=['time', 'signal', 'value'])
        self.saving = True
    
    def pause_saving(self):
        self.saving = False

    def resume_saving(self):
        self.saving = True
    
    def finish_saving(self, file_name):
        self.saving = False
        self.data.to_csv(file_name)


path = "/home/fernando/Dropbox/UC/Magister/robot-flautista/examples/bumblebee_rapido_2.json"
route = get_route_complete(path, go_back=False)
# for i in range(len(route['x'])):
#     route['x'][i] = x_units_to_mm(route['x'][i])

x = []
x_vel = []
z = []
z_vel = []
alpha = []
alpha_vel = []
for i in range(len(route['t'])):
    x.append((route['t'][i], route['x'][i]))
    x_vel.append((route['t'][i], route['x_vel'][i]))
    z.append((route['t'][i], route['z'][i]))
    z_vel.append((route['t'][i], route['z_vel'][i]))
    alpha.append((route['t'][i], route['alpha'][i]))
    alpha_vel.append((route['t'][i], route['alpha_vel'][i]))

# plt.plot(route['t'], route['x_vel'])
# plt.plot(route['t'], route['x'])
# plt.ylabel('X ref')
# plt.xlabel('Time')
# plt.show()
    
connected = True

x_event = threading.Event()
x_event.set()
x_driver = AMCIDriver('192.168.2.102', x_event, connected=connected, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], motors_step_turn=1000) # 10000 for alpha, 1000 for x and z
x_driver.start()

z_event = threading.Event()
z_event.set()
z_driver = AMCIDriver('192.168.2.104', z_event, connected=connected, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], motors_step_turn=1000)#, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'])
z_driver.start()

alpha_event = threading.Event()
alpha_event.set()
alpha_driver = AMCIDriver('192.168.2.103', alpha_event, connected=connected, starting_speed=1, motors_step_turn=10000)#, input_1_function_bits=INPUT_FUNCTION_BITS['Home'])
alpha_driver.start()

start_event = threading.Event()
t0 = time()

x_reference = Reference(x_event, start_event, x_driver, t0, acc=500, dec=500, proportional_coefficient=1, delay=0, move=True)
x_reference.positions = x
x_reference.velocities = x_vel
x_reference.start()

z_reference = Reference(z_event, start_event, z_driver, t0, acc=500, dec=500, proportional_coefficient=1, delay=0, move=True)
z_reference.positions = z
z_reference.velocities = z_vel
z_reference.start()

alpha_reference = Reference(alpha_event, start_event, alpha_driver, t0, acc=5000, dec=5000, proportional_coefficient=1, delay=0, move=True)
alpha_reference.positions = alpha
alpha_reference.velocities = alpha_vel
alpha_reference.start()

recorder = SimpleRecorder3(x_driver, z_driver, alpha_driver, x_reference, z_reference, alpha_reference, windowWidth=200, interval=10)

app = QApplication(sys.argv)
plotwin = Window3(app, recorder, x_reference, z_reference, alpha_reference, start_event, interval=10)
plotwin.show()
recorder.start()

sys.exit(app.exec())