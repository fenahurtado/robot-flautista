from utils.cinematica import *

from PyQt5.QtWidgets import QApplication, QHBoxLayout, QWidget, QVBoxLayout, QDialog

import numpy as np
import matplotlib.pyplot as plt

#from matplotlib.backends.backend_qt5agg import FigureCanvas
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

from matplotlib.figure import Figure


class RouteWidget(QDialog):
    def __init__(self, ri, thetai, oi, rf, thetaf, of, parent=None):
        super().__init__(parent)
        self.ri = ri
        self.thetai = thetai
        self.oi = oi
        self.rf = rf
        self.thetaf = thetaf
        self.of = of
        self.parent = parent

        self.lay = QVBoxLayout(self)

        self.canvas = FigureCanvas(Figure(figsize=(6, 6)))
        self.ax = self.canvas.figure.subplots()
        self.draw()

        self.toolbar = NavigationToolbar(self.canvas, self)
        self.lay.addWidget(self.toolbar)
        self.lay.addWidget(self.canvas)
    
    def draw(self):
        x_points, z_points, alpha_points, d = plan_route_min_error(self.ri, self.thetai, self.oi, self.rf, self.thetaf, self.of, error=0.05, plot=False)
        N = len(x_points)

        deltaR = self.rf - self.ri
        deltaTheta = self.thetaf - self.thetai
        deltaO = self.of - self.oi

        xi, zi, alphai = get_x_z_alpha(self.ri, self.thetai, self.oi)
        x2i, z2i = get_pos_punta(xi, zi, alphai)
        x_a, z_a, alpha_a = xi, zi, alphai

        xf, zf, alphaf = get_x_z_alpha(self.rf, self.thetaf, self.of)
        x2f, z2f = get_pos_punta(xf, zf, alphaf)

        d = 0
        x_points = []
        z_points = []
        alpha_points = []
        x2_points = []
        z2_points = []
        for n in range(N+1):
            xn, zn, alphan = get_x_z_alpha(self.ri + n*deltaR/N, self.thetai + n*deltaTheta/N, self.oi + n*deltaO/N)
            #print(ri + n*deltaR/N, thetai + n*deltaTheta/N, oi + n*deltaO/N)
            #print(get_r_theta_o(xn, zn, alphan))
            #print(alphan)
            x_points.append(xn)
            z_points.append(zn)
            alpha_points.append(alphan)
            x2, z2 = get_pos_punta(xn, zn, alphan)
            x2_points.append(x2)
            z2_points.append(z2)
            d += sqrt((xn - x_a)**2 + (zn - z_a)**2 + (alphan - alpha_a)**2)
            x_a, z_a, alpha_a = xn, zn, alphan

        self.ax.plot(x_points, z_points, color='b')
        self.ax.arrow(x_points[int(N/2)], z_points[int(N/2)], x_points[int(N/2)+1] - x_points[int(N/2)], z_points[int(N/2)+1] - z_points[int(N/2)], shape='full', lw=0, length_includes_head=True, head_width=2.5, color='b')
        self.ax.plot(x2_points, z2_points, color='b')
        self.ax.set_ylim ([30,210])
        self.ax.set_xlim([30,210])
        self.ax.invert_yaxis()
        circle1 = plt.Circle((DATA['X_F'], DATA['Z_F']+15), 15, color='y')
        self.ax.add_patch(circle1)
        circle2 = plt.Circle((DATA['X_F'], DATA['Z_F']), 2, color='cyan')
        self.ax.add_patch(circle2)
        self.ax.plot(xf, zf, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="green")
        self.ax.annotate(r'$(x_f, z_f)$', (xf, zf), textcoords="offset points", xytext=(0,10), ha='center', fontsize=15)
        self.ax.plot(x2f, z2f, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="green")
        self.ax.annotate(r'$(x_f\prime, z_f\prime)$', (x2f, z2f), textcoords="offset points", xytext=(0,10), ha='center', fontsize=15)
        self.ax.arrow(x2f, z2f, 8*cos(alphaf*pi/180), 8*sin(alphaf*pi/180), width=1, edgecolor='purple', facecolor='purple')

        self.ax.plot(xi, zi, marker="o", markersize=5, markeredgecolor="red", markerfacecolor="red")
        self.ax.annotate(r'$(x_i, z_i)$', (xi, zi), textcoords="offset points", xytext=(0,-20), ha='center', fontsize=15)
        self.ax.plot(x2i, z2i, marker="o", markersize=5, markeredgecolor="red", markerfacecolor="red")
        self.ax.annotate(r'$(x_i\prime, z_i\prime)$', (x2i, z2i), textcoords="offset points", xytext=(0,-20), ha='center', fontsize=15)
        self.ax.arrow(x2i, z2i, 8*cos(alphai*pi/180), 8*sin(alphai*pi/180), width=1, edgecolor='purple', facecolor='purple')

    def redraw(self, ri, thetai, oi, rf, thetaf, of):
        self.ri = ri
        self.thetai = thetai
        self.oi = oi
        self.rf = rf
        self.thetaf = thetaf
        self.of = of
        self.clean()
        self.draw()
        self.canvas.draw()

    def clean(self):
        self.ax.cla()
        self.canvas.draw() 

    def closeEvent(self, event):
        self.parent.route_window = None

class RampWidget(QDialog):
    def __init__(self, ref_i, ref_f, vibrato_amp, vibrato_freq, deformation, T, parent=None):
        super().__init__(parent)

        self.ref_i = ref_i
        self.ref_f = ref_f
        self.vibrato_amp = vibrato_amp
        self.vibrato_freq = vibrato_freq
        self.deformation = deformation
        self.T = T
        self.parent = parent

        self.lay = QVBoxLayout(self)

        self.canvas = FigureCanvas(Figure(figsize=(6, 6)))
        self.ax = self.canvas.figure.subplots()

        time = np.linspace(0,self.T,100)

        vibrato = self.vibrato_amp * np.sin(time * 2*np.pi * self.vibrato_freq)
        ramp = self.ref_i + (self.ref_f-self.ref_i) * (time / self.T) ** self.deformation
        refs = ramp + vibrato
        refs = np.where(refs > 50, 50, refs)
        refs = np.where(refs < 0, 0, refs)
 
        #refs = np.where(time > self.delay, self.ramp(time), self.ref_i)
        #refs = np.where(time < self.T - self.lead, refs, self.ref_f)
       
        self.ax.plot(time, refs, color="b")

        self.toolbar = NavigationToolbar(self.canvas, self)
        self.lay.addWidget(self.toolbar)
        self.lay.addWidget(self.canvas)

    def clean(self):
        self.ax.cla()
        self.canvas.draw() 
        #QApplication.processEvents()
    
    def redraw(self, ref_f, vibrato_amp, vibrato_freq, deformation, T):
        self.clean()

        self.ref_f = ref_f
        self.vibrato_amp = vibrato_amp
        self.vibrato_freq = vibrato_freq
        self.deformation = deformation
        self.T = T

        time = np.linspace(0,self.T,100)

        vibrato = self.vibrato_amp * np.sin(time * 2*np.pi * self.vibrato_freq)
        ramp = self.ref_i + (self.ref_f-self.ref_i) * (time / self.T) ** self.deformation
        refs = ramp + vibrato
        refs = np.where(refs > 50, 50, refs)
        refs = np.where(refs < 0, 0, refs)

        # refs = np.where(time > self.delay, self.ramp(time), self.ref_i)
        # refs = np.where(time < self.T - self.lead, refs, self.ref_f)
       
        self.ax.plot(time, refs, color="b")
        self.canvas.draw()

    def ramp(self, val):
        return self.ref_i + (self.ref_f-self.ref_i) * ((val-self.delay) / (self.T-(self.delay + self.lead)))**self.deformation

    def closeEvent(self, event):
        self.parent.ramp_window = None