from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox, QFileDialog, QLineEdit, QPushButton, QVBoxLayout
    )
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore

import sys
from time import time, sleep
from numpy import linspace
from random import random, randint
from functools import partial

from views.plot_window import Ui_MainWindow as PlotWindow

signals = ['Radius', 'Incidence Angle', 'Jet Offset', 'Position', 'Mouth Pressure', 'Mass Flow Rate', 'Volume Flow Rate', 'Air Temperature', 'Sound Frequency', 'X Position', 'Z Position', 'Alpha Position']

class Window(QMainWindow, PlotWindow, QtCore.QThread):
    def __init__(self, app, measure, data, interval=10, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        self.interval = interval
        self.parent = parent
        self.app = app
        self.measures = [measure]
        self.traces = []
        if measure != 3:
            for i in range(len(signals)):
                if measure != i and i != 3:
                    self.traces.append(self.menuAdd_Trace.addAction(signals[i]))
                    self.traces[-1].triggered.connect(partial(self.add_trace, i))
        self.data = data

        #self.ptr = -self.windowWidth                      # set first x position
        self.curves = []
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k', 'w', (randint(0,255),randint(0,255),randint(0,255)), (randint(0,255),randint(0,255),randint(0,255)), (randint(0,255),randint(0,255),randint(0,255)), (randint(0,255),randint(0,255),randint(0,255))]

        for i in range(len(signals)):
            self.curves.append(self.graphicsView.plot(pen=pg.mkPen(colors[i], width=1)))
        self.ref_curve = self.graphicsView.plot(pen=pg.mkPen('w', width=1, style=QtCore.Qt.DashLine))
        if measure == 3:
            self.graphicsView.setXRange(-1, 110, padding=0)
            self.graphicsView.setYRange(-1, 110, padding=0)
        
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.interval)

        self.t0 = time()
        self.t = 0
    
    def add_trace(self, index):
        print(index)

    def update(self):
        for index in range(len(self.measures)):
            if self.measures[index] == 0: # Radius
                self.curves[index].setData(self.data.times, self.data.radius)
            elif self.measures[index] == 1: # Theta
                self.curves[index].setData(self.data.times, self.data.theta)
            elif self.measures[index] == 2: # Offset
                self.curves[index].setData(self.data.times, self.data.offset)
            elif self.measures[index] == 3: # Position
                self.curves[index].setData(self.data.x, self.data.z)
            elif self.measures[index] == 4: # Mouth Pressure
                self.curves[index].setData(self.data.times, self.data.mouth_pressure)
            elif self.measures[index] == 5: # Mass Flow
                self.curves[index].setData(self.data.times, self.data.mass_flow)
                self.ref_curve.setData(self.data.times, self.data.flow_ref)
            elif self.measures[index] == 6: # Volume Flow
                self.curves[index].setData(self.data.times, self.data.volume_flow)
            elif self.measures[index] == 7: # Temperature
                self.curves[index].setData(self.data.times, self.data.temperature)
            elif self.measures[index] == 8: # Frequency
                self.curves[index].setData(self.data.times, self.data.frequency)
            elif self.measures[index] == 9: # X
                self.curves[index].setData(self.data.times, self.data.x)
                self.ref_curve.setData(self.data.times, self.data.x_ref)
            elif self.measures[index] == 10: # Z
                self.curves[index].setData(self.data.times, self.data.z)
                self.ref_curve.setData(self.data.times, self.data.z_ref)
            elif self.measures[index] == 11: # Alpha
                self.curves[index].setData(self.data.times, self.data.alpha)
                self.ref_curve.setData(self.data.times, self.data.alpha_ref)
            
        self.app.processEvents()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window(app)
    win.show()

    sys.exit(app.exec())

# Import libraries
# from numpy import *
# from pyqtgraph.Qt import QtGui, QtCore
# import pyqtgraph as pg
# from random import random

# ### START QtApp #####
# app = QtGui.QApplication([])            # you MUST do this once (initialize things)
# ####################

# win = pg.GraphicsWindow(title="Signal from serial port") # creates a window
# p = win.addPlot(title="Realtime plot")  # creates empty space for the plot in the window
# curve = p.plot()                        # create an empty "plot" (a curve to plot)

# windowWidth = 500                       # width of the window displaying the curve
# Xm = linspace(0,0,windowWidth)          # create array that will contain the relevant time series     
# ptr = -windowWidth                      # set first x position

# # Realtime data plot. Each time this function is called, the data display is updated
# def update():
#     global curve, ptr, Xm    
#     Xm[:-1] = Xm[1:]                      # shift data in the temporal mean 1 sample left
#     value = 10 * random()                 # read line (single value) from the serial port
#     Xm[-1] = float(value)                 # vector containing the instantaneous values      
#     ptr += 1                              # update x position for displaying the curve
#     curve.setData(Xm)                     # set the curve with this data
#     curve.setPos(ptr,0)                   # set x position in the graph to 0
#     QtGui.QApplication.processEvents()    # you MUST process the plot now

# ### MAIN PROGRAM #####    
# # this is a brutal infinite loop calling your realtime data plot
# while True: update()

# ### END QtApp ####
# pg.QtGui.QApplication.exec_() # you MUST put this at the end
# ##################


# self.timer = QtCore.QTimer()
# self.timer.timeout.connect(self.update)
# self.timer.start(self.interval)