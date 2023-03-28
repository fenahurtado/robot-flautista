import sys
sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtCore
import pyqtgraph as pg
import lib.ethernet_ip.ethernetip as ethernetip

from utils.cinematica import *
from utils.motor_route import *
from exercises.drivers_connect import AMCIDriver, INPUT_FUNCTION_BITS, VirtualAxis, Musician
#from view_control.plot_window import Window as PlotWindow
from views.simple_plot_window import Ui_MainWindow as PlotWindowView
from pyqtgraph.Qt import QtGui

import matplotlib.pyplot as plt
from time import time
from datetime import datetime
import pandas as pd


class Window(QMainWindow, PlotWindowView):
    def __init__(self, app, running, data, t0, x_reference, z_reference, alpha_reference, x_data, z_data, alpha_data, interval=10, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.interval = interval
        self.parent = parent
        self.running = running
        self.x_reference = x_reference
        self.z_reference = z_reference
        self.alpha_reference = alpha_reference
        self.x_data = x_data
        self.z_data = z_data
        self.alpha_data = alpha_data
        #self.start_event = start_event
        self.app = app
        self.data = data
        self.curves = [self.graphicsView.plot(pen=pg.mkPen('b', width=1)), 
                       self.graphicsView.plot(pen=pg.mkPen('r', width=1, style=QtCore.Qt.DashLine)),
                       self.graphicsView.plot(pen=pg.mkPen('g', width=1)),
                       self.graphicsView.plot(pen=pg.mkPen('w', width=1, style=QtCore.Qt.DashLine)),
                       self.graphicsView.plot(pen=pg.mkPen('c', width=1)),
                       self.graphicsView.plot(pen=pg.mkPen('y', width=1, style=QtCore.Qt.DashLine))]
        
        #self.x_reference.finish_score_signal.connect(self.stop_action)
        self.startButton.clicked.connect(self.start_action)
        self.stopButton.clicked.connect(self.stop_action)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.interval)
        self.plot_choise = 0
        self.xButton.toggled.connect(self.plot_x)
        self.zButton.toggled.connect(self.plot_z)
        self.alphaButton.toggled.connect(self.plot_alpha)
        self.recordCheckBox.toggled.connect(self.change_record_data)
        self.record_data = False
        self.t0 = t0
        self.t = 0
    
    def closeEvent(self, a0: QtGui.QCloseEvent):
        '''
        Esta función se ejecuta al cerrar el programa, para terminar todos los threads que están corriendo
        '''
        self.running.clear()
        return super().closeEvent(a0)

    def change_record_data(self, value):
        self.record_data = value

    def plot_x(self, value):
        if value:
            self.plot_choise = 0
        else:
            self.curves[0].setData([], [])
            self.curves[1].setData([], [])

    def plot_z(self, value):
        if value:
            self.plot_choise = 1
        else:
            self.curves[2].setData([], [])
            self.curves[3].setData([], [])

    def plot_alpha(self, value):
        if value:
            self.plot_choise = 2
        else:
            self.curves[4].setData([], [])
            self.curves[5].setData([], [])

    def start_action(self):
        t = time() - self.t0
        self.data.start_saving()
        x = []
        z = []
        alpha = []
        for i in range(len(self.x_data)):
            x.append([self.x_data[i][0] + t, self.x_data[i][1], self.x_data[i][2]])
            z.append([self.z_data[i][0] + t, self.z_data[i][1], self.z_data[i][2]])
            alpha.append([self.alpha_data[i][0] + t, self.alpha_data[i][1], self.alpha_data[i][2]])
        #print(x)
        self.x_reference.merge_ref(x)
        self.z_reference.merge_ref(z)
        self.alpha_reference.merge_ref(alpha)

    def stop_action(self):
        #if self.start_event.is_set():
        filename = "/home/fernando/Dropbox/UC/Magister/robot-flautista/exercises/data/" + datetime.now().strftime("%d-%m-%Y_%H:%M:%S") + ".csv"
        self.data.finish_saving(filename)
        #self.start_event.clear()
        

    def update(self):
        if self.plot_choise == 0:
            self.curves[0].setData(self.data.times, self.data.x)
            self.curves[1].setData(self.data.times, self.data.x_ref)
        elif self.plot_choise == 1:
            self.curves[2].setData(self.data.times, self.data.z)
            self.curves[3].setData(self.data.times, self.data.z_ref)
        elif self.plot_choise == 2:
            self.curves[4].setData(self.data.times, self.data.alpha)
            self.curves[5].setData(self.data.times, self.data.alpha_ref)

        self.app.processEvents()

class SimpleRecorder:
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

        self.ref_state_x = x_units_to_mm(self.x_reference.pos)
        self.real_state_x = x_units_to_mm(self.x_driver.motor_position)
        self.ref_state_z = z_units_to_mm(self.z_reference.pos)
        self.real_state_z = z_units_to_mm(self.z_driver.motor_position)
        self.ref_state_alpha = alpha_units_to_angle(self.alpha_reference.pos)
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
        self.ref_state_x = x_units_to_mm(self.x_reference.pos)
        self.vel_state_x = self.x_reference.vel
        self.real_state_x = x_units_to_mm(self.x_driver.motor_position)
        self.ref_state_z = z_units_to_mm(self.z_reference.pos)
        self.vel_state_z = self.z_reference.vel
        self.real_state_z = z_units_to_mm(self.z_driver.motor_position)
        self.ref_state_alpha = alpha_units_to_angle(self.alpha_reference.pos)
        self.vel_state_alpha = self.alpha_reference.vel
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
            new_data = pd.DataFrame({'time': [t, t, t, t, t, t, t, t, t], 'signal':['x_ref', 'x', 'z_ref', 'z', 'alpha_ref', 'alpha', 'vel_ref_x', 'vel_ref_z', 'vel_ref_alpha'], 'value': [self.x_ref[-1], self.x[-1], self.z_ref[-1], self.z[-1], self.alpha_ref[-1], self.alpha[-1], self.vel_state_x, self.vel_state_z, self.vel_state_alpha]})
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


path = "/home/fernando/Dropbox/UC/Magister/robot-flautista/exercises/bumblebee_ejercicio.json"
route = get_route_complete(path, go_back=False)
# for i in range(len(route['x'])):
#     route['x'][i] = x_units_to_mm(route['x'][i])

x = []
z = []
alpha = []
for i in range(len(route['t'])):
    x.append([route['t'][i], route['x'][i], route['x_vel'][i]])
    z.append([route['t'][i], route['z'][i], route['z_vel'][i]])
    alpha.append([route['t'][i], route['alpha'][i], route['alpha_vel'][i]])


# plt.plot(route['t'], route['x_vel'])
# plt.plot(route['t'], route['x'])
# plt.ylabel('X ref')
# plt.xlabel('Time')
# plt.show()
    
connected = True
# running = threading.Event()
# running.set()
t0 = time()

host = "192.168.2.10"
connections = ["192.168.2.102", "192.168.2.104", "192.168.2.103", "192.168.2.101", "192.168.2.100"]
EIP = ethernetip.EtherNetIP(host)

interval = 0.01

# x_virtual_axis = VirtualAxis(running, interval, t0, verbose=False)
# z_virtual_axis = VirtualAxis(running, interval, t0, verbose=False)
# alpha_virtual_axis = VirtualAxis(running, interval, t0, verbose=False)
# x_virtual_axis.start()
# z_virtual_axis.start()
# alpha_virtual_axis.start()
# #virtual_flow = VirtualFlow(running, interval, t0, verbose=False)

# x_driver = AMCIDriver(EIP, connections[0], running, x_virtual_axis, connected=True, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], virtual_axis_follow_acceleration=50, virtual_axis_follow_deceleration=50, home=True)
# z_driver = AMCIDriver(EIP, connections[1], running, z_virtual_axis, connected=True, starting_speed=1, verbose=False, input_1_function_bits=INPUT_FUNCTION_BITS['CCW Limit'], virtual_axis_follow_acceleration=50, virtual_axis_follow_deceleration=50, home=False)
# alpha_driver = AMCIDriver(EIP, connections[2], running, alpha_virtual_axis, connected=True, starting_speed=1, verbose=False, motors_step_turn=10000, virtual_axis_follow_acceleration=500, virtual_axis_follow_deceleration=500, home=False)
# x_driver.start()
# z_driver.start()
# alpha_driver.start()

host = "192.168.2.10"
connections = ["192.168.2.102", "192.168.2.104", "192.168.2.103", "192.168.2.101", "192.168.2.100"]
event = threading.Event()
event.set()

t0 = time()
pierre = Musician(host, connections, event, home=False, x_connect=True, z_connect=True, alpha_connect=True, fingers_connect=False)
pierre.start()

recorder = SimpleRecorder(pierre.x_driver, pierre.z_driver, pierre.alpha_driver, pierre.x_virtual_axis, pierre.z_virtual_axis, pierre.alpha_virtual_axis, windowWidth=200, interval=10)

app = QApplication(sys.argv)
plotwin = Window(app, event, recorder, t0, pierre.x_virtual_axis, pierre.z_virtual_axis, pierre.alpha_virtual_axis, x, z, alpha, interval=10)
plotwin.show()
recorder.start()

sys.exit(app.exec())