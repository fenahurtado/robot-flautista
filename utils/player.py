import sys
import threading
from time import time, sleep
from PyQt5.QtWidgets import QApplication
from PyQt5 import QtCore
import numpy as np

from utils.driver_amci import AMCIDriver
from utils.microphone import Microphone
from utils.sensores_alicat import FlowController, PreasureSensor
from view_control.amci_control import AMCIWidget
from view_control.main_window import Window

from utils.cinematica import *


class MotorsController(threading.Thread):
    '''
    Esta clase se usa para controlar el movimiento de los motores. Desde cualquier otro thread se puede llamar a la función move_to con un estado final al que se quiere llegar, aceleración y tiempo; y el thread del controlador conseguirá que los motores se coordinen para llegar a tal posición.
    '''
    def __init__(self, running, state, x_driver, z_driver, alpha_driver):
        threading.Thread.__init__(self)
        self.state = state
        self.running = running
        self.x_driver = x_driver
        self.z_driver = z_driver
        self.alpha_driver = alpha_driver
        self.changeEvent = threading.Event()
        self.stop_movement = False
        self.only_cartesians = False
        self.route = []
        self.x_ref = self.state.x
        self.z_ref = self.state.z
        self.alpha_ref = self.state.alpha
        self.programmed_speed = 1000
        self.x_driver.motor_position_signal.connect(self.change_x_position)
        self.z_driver.motor_position_signal.connect(self.change_z_position)
        self.alpha_driver.motor_position_signal.connect(self.change_alpha_position)

    def change_x_position(self, value):
        '''
        Actualiza la posición en x de acuerdo a lo informado por el driver
        '''
        self.state.x = self.x_units_to_mm(value)

    def change_z_position(self, value):
        '''
        Actualiza la posición en z de acuerdo a lo informado por el driver
        '''
        self.state.z = self.z_units_to_mm(value)

    def change_alpha_position(self, value):
        '''
        Actualiza la posición en alpha de acuerdo a lo informado por el driver
        '''
        self.state.alpha = self.alpha_units_to_angle(value)

    def x_mm_to_units(self, mm):
        '''
        Transforma los mm de avance en X a pasos para el controlador
        input: mm = milímetros en el eje X
        output: pasos para el motor
        '''
        if self.x_driver.motors_step_turn:
            return int(mm * self.x_driver.motors_step_turn / 8 )
        return int(mm * 1000 / 8 )
    
    def x_units_to_mm(self, units):
        '''
        Transforma los pasos del controlador en mm de avance en X
        input: units = pasos para el motor
        output: milímetros en el eje X
        '''
        if self.x_driver.motors_step_turn:
            return units * 8 / self.x_driver.motors_step_turn
        return units * 8 / 1000

    def z_mm_to_units(self, mm):
        '''
        Transforma los mm de avance en Z a pasos para el controlador
        input: mm = milímetros en el eje Z
        output: pasos para el motor
        '''
        if self.z_driver.motors_step_turn:
            return int(mm * self.z_driver.motors_step_turn / 8 )
        return int(mm * 1000 / 8 )

    def z_units_to_mm(self, units):
        '''
        Transforma los pasos del controlador en mm de avance en Z
        input: units = pasos para el motor
        output: milímetros en el eje Z
        '''
        if self.z_driver.motors_step_turn:
            return units * 8 / self.z_driver.motors_step_turn
        return units * 8 / 1000
    
    def alpha_angle_to_units(self, angle):
        '''
        Transforma los grados de avance en alpha a pasos para el controlador
        input: angle = grados el eje Alpha
        output: pasos para el motor
        '''
        if self.alpha_driver.motors_step_turn:
            return int(angle * self.alpha_driver.motors_step_turn / 360)
        return int(angle * 10000 / 360)
    
    def alpha_units_to_angle(self, units):
        '''
        Transforma los pasos del controlador en grados de avance en alpha
        input: units = pasos para el motor
        output: grados el eje Alpha
        '''
        if self.alpha_driver.motors_step_turn:
            return units * 360 / self.alpha_driver.motors_step_turn
        return units * 360 / 10000

    def get_route_positions(self, xi, zi, alphai, xf, zf, alphaf, divisions=20, plot=False):
        '''
        Planifica una ruta desde un punto de inicio (xi, zi, alphai) hasta un punto final (xf, zf, alphaf) pasando por la trayectoria de las variables del sistema: r, theta y offset. Es decir, si se quiere ir desde un punto de inicio A que está a radio r_a de la flauta a un punto B que está a radio r_b (pero el ángulo theta y el offset se mantienen), se calcula la trayectoria de forma que las otras variables (theta y offset) se mantengan constantes en todo su recorrido. En el caso del radio esta trayectoria es recta, pero en el de theta es circular y en el del offset elíptico. También se permite una combinación de estas tres.

        inputs:
        xi, zi, alphai = posición inicial
        xf, zf, alphaf = posición final
        divisions      = cantidad de puntos intermedios
        plot           = True si se quiere desplegar el plot de la trayectoria

        outputs:
        x_points     = lista de posiciónes en el eje x
        z_points     = lista de posiciónes en el eje z
        alpha_points = lista de posiciónes en el eje alpha
        d            = lista de la distancia recorrida hasta cada punto
        '''
        ri, thetai, oi = get_r_theta_o(xi, zi, alphai)
        rf, thetaf, of = get_r_theta_o(xf, zf, alphaf)

        deltaR = rf - ri
        deltaTheta = thetaf - thetai
        deltaO = of - oi

        x2i, z2i = get_pos_punta(xi, zi, alphai*pi/180)
        x_a, z_a, alpha_a = xi, zi, alphai

        x2f, z2f = get_pos_punta(xf, zf, alphaf*pi/180)

        dist = 0
        d = []
        x_points = []
        z_points = []
        alpha_points = []
        x2_points = []
        z2_points = []
        for n in range(divisions+1):
            xn, zn, alphan = get_x_z_alpha(ri + n*deltaR/divisions, thetai + n*deltaTheta/divisions, oi + n*deltaO/divisions)
            #print(ri + n*deltaR/N, thetai + n*deltaTheta/N, oi + n*deltaO/N)
            #print(get_r_theta_o(xn, zn, alphan))
            #print(alphan)
            x_points.append(round(xn,3))
            z_points.append(round(zn,3))
            alpha_points.append(round(alphan,3))
            if plot:
                x2, z2 = get_pos_punta(xn, zn, alphan*pi/180)
                x2_points.append(x2)
                z2_points.append(z2)
            dist += sqrt((xn - x_a)**2 + (zn - z_a)**2 + (alphan - alpha_a)**2)
            d.append(dist)
            x_a, z_a, alpha_a = xn, zn, alphan

        if plot:
            fig, ax = plt.subplots(figsize=(6,6))
            ax.plot(x_points, z_points, color='b')
            ax.arrow(x_points[int(divisions/2)], z_points[int(divisions/2)], x_points[int(divisions/2)+1] - x_points[int(divisions/2)], z_points[int(divisions/2)+1] - z_points[int(divisions/2)], shape='full', lw=0, length_includes_head=True, head_width=2.5, color='b')
            ax.plot(x2_points, z2_points, color='b')
            ax.set_ylim ([30,210])
            ax.set_xlim([30,210])
            ax.invert_yaxis()
            circle1 = plt.Circle((DATA["X_F"], DATA["Z_F"]+15), 15, color='y')
            ax.add_patch(circle1)
            circle2 = plt.Circle((DATA["X_F"], DATA["Z_F"]), 2, color='cyan')
            ax.add_patch(circle2)
            ax.plot(xf, zf, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="green")
            ax.annotate(r'$(x_f, z_f)$', (xf, zf), textcoords="offset points", xytext=(0,10), ha='center', fontsize=15)
            ax.plot(x2f, z2f, marker="o", markersize=5, markeredgecolor="green", markerfacecolor="green")
            ax.annotate(r'$(x_f\prime, z_f\prime)$', (x2f, z2f), textcoords="offset points", xytext=(0,10), ha='center', fontsize=15)
            ax.arrow(x2f, z2f, 8*cos(alphaf*pi/180), 8*sin(alphaf*pi/180), width=1, edgecolor='purple', facecolor='purple')

            ax.plot(xi, zi, marker="o", markersize=5, markeredgecolor="red", markerfacecolor="red")
            ax.annotate(r'$(x_i, z_i)$', (xi, zi), textcoords="offset points", xytext=(0,-20), ha='center', fontsize=15)
            ax.plot(x2i, z2i, marker="o", markersize=5, markeredgecolor="red", markerfacecolor="red")
            ax.annotate(r'$(x_i\prime, z_i\prime)$', (x2i, z2i), textcoords="offset points", xytext=(0,-20), ha='center', fontsize=15)
            ax.arrow(x2i, z2i, 8*cos(alphai*pi/180), 8*sin(alphai*pi/180), width=1, edgecolor='purple', facecolor='purple')
            plt.show()

        return x_points, z_points, alpha_points, d

    def max_dist_rec(self, acc, dec, T):
        '''
        Dice cuál es la mayor distancia que se puede recorrer durante un tiempo T con la aceleración y desaceleración descritas.
        Sirve para validar si es posible realizar un movimiento (supone que el motor es capaz de lograr esas aceleraciones y que no tiene límite de velocidad)
        
        inputs:
        acc = aceleración de partida
        dec = desaceleración de frenado
        T   = tiempo en movimiento

        output:
        dist_max = distancia máxima
        '''
        d_acc = (acc/2) * ((dec*T)/(acc+dec))**2
        d_dec = acc*(dec*T)/(acc+dec) * (T-(dec*T)/(acc+dec)) / 2
        dist_max = d_acc + d_dec
        return dist_max

    def plan_speed_curve(self, d, acceleration, deceleration, T):
        '''
        A partir de una distancia d que se quiere recorrer en un tiempo T con una aceleración de partida y una de detención descritas, se planifica una curva de velocidades con forma trapezoidal.
        
        inputs:
        d            = distancia total que se quiere recorrer
        acceleration = aceleración de partida
        deceleration = desaceleración de frenado
        T            = tiempo en movimiento

        outputs:
        speed = velocidad en regimen permanente
        t_acc = tiempo durante el que acelera
        t_dec = tiempo en el que empieza a frenar
        '''
        #print(d, T, acceleration, deceleration, self.max_dist_rec(acceleration, deceleration, T))
        speed = (acceleration*deceleration*T - sqrt(acceleration*deceleration*(acceleration*deceleration*T**2 - 2*acceleration*d - 2*deceleration*d))) / (acceleration+deceleration)
        t_acc = speed / acceleration
        t_dec = T - speed / deceleration
        return speed, t_acc, t_dec

    def plan_temps_according_to_speed(self, distances, vel, t_acc, t_dec, acc, dec):
        '''
        Entrega una lista con los tiempos en los que se debe pasar por cada punto de la ruta de acuerdo a la curva de velocidades que se quiere lograr.

        inputs:
        distances = lista de la distancia recorrida hasta cada punto
        vel       = velocidad en regimen permanente
        t_acc     = tiempo durante el que acelera
        t_dec     = tiempo en el que empieza a frenar
        acc       = aceleración de partida
        dec       = desaceleración de frenado

        outputs:
        temps = lista de tiempos en los que se quiere pasar por cada punto de la trayectoria
        '''
        d_t_acc = acc * t_acc**2 / 2
        d_t_dec = d_t_acc + vel * (t_dec - t_acc)
        temps = []
        for d_sum in distances:
            if d_sum < d_t_acc:
                temps.append(sqrt(2*d_sum/acc))
            elif d_sum < d_t_dec:
                temps.append((d_sum - d_t_acc)/vel + t_acc)
            else:
                a = dec / 2
                b = -(vel + (2*t_dec*dec)/2)
                c = d_sum - d_t_dec + vel*t_dec + (dec*t_dec**2)/2
                t = (-b - sqrt(round(b**2 - 4*a*c,3)))/(2*a)
                temps.append(t)
        return temps

    def plan_route(self, x_points, z_points, alpha_points, temps):
        '''
        Recopila las listas de posiciones y tiempos, con ellas calcula las velocidades (con el método de las secantes) para cada eje y devuelve una lista con diccionarios para cada punto.

        inputs:
        x_points     = lista de posiciones en x
        z_points     = lista de posiciones en z
        alpha_points = lista de posiciones en alpha
        temps        = lista de tiempos en los que se quiere pasar por cada punto de la trayectoria

        output:
        steps = diccionario con las instrucciones de los assemblies para cada eje más una lista de los tiempos en los que se debe pasar por cada referencia
        '''
        steps = {'x': [], 'z': [], 'alpha': [], 't': []}

        for i in range(len(x_points) - 1):
            x = self.x_mm_to_units(x_points[i])            
            z = self.z_mm_to_units(z_points[i])
            alpha = self.alpha_angle_to_units(alpha_points[i])
            t = temps[i]

            x_f = self.x_mm_to_units(x_points[i+1])
            z_f = self.z_mm_to_units(z_points[i+1])
            alpha_f = self.alpha_angle_to_units(alpha_points[i+1])
            t_f = temps[i+1]

            x_step = {'pos': int(x_f - x), 'speed': max(1,int(abs(x_f - x) / (t_f - t))), 'acc': 100, 'dec': 100, 'jerk': 0}
            z_step = {'pos': int(z_f - z), 'speed': max(1,int(abs(z_f - z) / (t_f - t))), 'acc': 100, 'dec': 100, 'jerk': 0}
            alpha_step = {'pos': int(alpha_f - alpha), 'speed': max(1,int(abs(alpha_f - alpha) / (t_f - t))), 'acc': 5000, 'dec': 5000, 'jerk': 0}
            
            steps['x'].append(x_step)
            steps['z'].append(z_step)
            steps['alpha'].append(alpha_step)
            steps['t'].append(t_f - t)

        #print(steps['x'])
        return steps

    def move_to(self, desired_state, acc=20, dec=20, T=None):
        '''
        Función para llamar desde cualquier thread. Setea los parámetros necesarios para que después se ejecute (desde el thread del controlador) la trayectoria desde el estado actual hasta desired_state siguiendo las trayectorias del sistema

        inputs:
        desired_state = estado al que se quiere llegar. De la clase State
        acc           = aceleración de partida
        dec           = desaceleración de frenado
        T             = tiempo en el que se quiere realizar el movimiento

        output:
        T  = tiempo en el que se quiere realizar el movimiento (si no se da se calcula según la distancia a recorrer)
        '''
        if self.state.is_too_close(desired_state):
            return 0

        x_points, z_points, alpha_points, d = self.get_route_positions(*self.state.cart_coords(), *desired_state.cart_coords(), divisions=12, plot=False)

        #print(x_points)
        
        if not T:
            T = 0.1
            while True:
                if not self.max_dist_rec(acc, dec, T) < d[-1]:
                    break
                T += 0.1
            
            T = T*2
        else:
            if self.max_dist_rec(acc, dec, T) < d[-1]:
                print(f'Impossible to achieve such position with given acceleration and deceleration. {d[-1]} > {self.max_dist_rec(acc, dec, T)}')
                return

        vel, t_acc, t_dec = self.plan_speed_curve(d[-1], acc, dec, T)
        temps = self.plan_temps_according_to_speed(d, vel, t_acc, t_dec, acc, dec)
        self.route = self.plan_route(x_points, z_points, alpha_points, temps)

        if self.changeEvent.is_set():
            self.stop()

        self.changeEvent.set()

        return T

    def move_cartesians_only(self, desired_state, speed=1000):
        '''
        Función para llamar desde cualquier thread. Setea los parámetros necesarios para que después se ejecute (desde el thread del controlador) la trayectoria desde el estado actual hasta desired_state siguiendo trayectorias cartesianas.
        '''
        #print(desired_state)
        self.x_ref = desired_state.x
        self.z_ref = desired_state.z
        self.alpha_ref = desired_state.alpha
        self.programmed_speed = speed
        self.only_cartesians = True
        self.stop()

        self.changeEvent.set()

    def home_alpha(self):
        '''
        Función que setea la posición actual del eje alpha como el cero
        '''
        self.alpha_driver.request_write_preset_position(0)
        self.alpha_ref = 0
        self.state.alpha = 0

    def move_alpha(self, value):
        '''
        Función que mueve el eje alpha a la posición indicada en value
        
        input:
        value = posición en grados a la que se quiere mover el eje
        '''
        units = self.alpha_angle_to_units(value)
        self.alpha_driver.request_write_absolute_move(units, programmed_speed=10000, acceleration=1000, deceleration=1000)
        self.alpha_driver.request_write_return_to_command_mode()

    def stop(self):
        '''
        Función que se puede llamar desde otro thread para frenar un movimiento en acción
        '''
        self.stop_movement = True
        if not self.x_driver.stopped:
            self.x_driver.request_write_hold_move()
        if not self.z_driver.stopped:
            self.z_driver.request_write_hold_move()
        if not self.alpha_driver.stopped:
            self.alpha_driver.request_write_hold_move()
        #self.changeEvent.set()

    def homed(self):
        '''
        Establece la posición actual de los tres motores como el cero
        '''
        self.x_driver.request_write_preset_position(0)
        self.x_ref = 0
        self.state.x = 0
        self.z_driver.request_write_preset_position(0)
        self.z_ref = 0
        self.state.z = 0
        self.alpha_driver.request_write_preset_position(0)
        self.alpha_ref = 0
        self.state.alpha = 0

    def reset_drivers(self):
        '''
        Resetea los drivers a sus configuraciones iniciales.
        '''
        self.x_driver.request_write_reset()
        self.z_driver.request_write_reset()
        self.alpha_driver.request_write_reset()
        
    def run(self):
        while self.running.is_set():
            self.changeEvent.wait(timeout=1)
            if self.stop_movement:
                stopped = True
                if not self.x_driver.stopped:
                    stopped = False
                    self.x_driver.request_write_hold_move()
                if not self.z_driver.stopped:
                    stopped = False
                    self.z_driver.request_write_hold_move()
                if not self.alpha_driver.stopped:
                    stopped = False
                    self.alpha_driver.request_write_hold_move()
                self.stop_movement = False
                if not stopped:
                    sleep(0.4)
                #self.changeEvent.clear()
            if self.changeEvent.is_set():
                self.x_driver.request_write_reset_errors()
                self.z_driver.request_write_reset_errors()
                self.alpha_driver.request_write_reset_errors()
                if not self.only_cartesians:
                    # self.changeEvent.clear()
                    # continue
                    self.x_driver.request_program_run_assembled_move(self.route['x'], blend_direction=0, dwell_move=1)
                    self.z_driver.request_program_run_assembled_move(self.route['z'], blend_direction=0, dwell_move=1)
                    self.alpha_driver.request_program_run_assembled_move(self.route['alpha'], blend_direction=0, dwell_move=1)

                    self.x_ref = self.state.x
                    self.z_ref = self.state.z
                    self.alpha_ref = self.state.alpha
                    for step in range(len(self.route['x'])):
                        sleep(self.route['t'][step])
                        self.x_ref += self.x_units_to_mm(self.route['x'][step]['pos'])
                        self.z_ref += self.z_units_to_mm(self.route['z'][step]['pos'])
                        self.alpha_ref += self.alpha_units_to_angle(self.route['alpha'][step]['pos'])
                        if self.stop_movement:
                            break
                        self.changeEvent.clear()

                else:
                    self.x_driver.request_write_absolute_move(self.x_mm_to_units(self.x_ref), programmed_speed=self.programmed_speed, acceleration=1000, deceleration=1000)
                    self.z_driver.request_write_absolute_move(self.z_mm_to_units(self.z_ref), programmed_speed=self.programmed_speed, acceleration=1000, deceleration=1000)
                    self.alpha_driver.request_write_absolute_move(self.alpha_angle_to_units(self.alpha_ref), programmed_speed=self.programmed_speed*10, acceleration=1000, deceleration=1000)

                    self.x_driver.request_write_return_to_command_mode()
                    self.z_driver.request_write_return_to_command_mode()
                    self.alpha_driver.request_write_return_to_command_mode()

                    self.only_cartesians = False
                    self.changeEvent.clear()
        
        print('Signal ended')

class Recorder:
    '''
    Esta clase se encarga de almacenar la historia de las variables medidas. windowWidth dice la cantidad de datos a almacenar e interval el tiempo (en milisegundos) para obtener una muestra.
    '''
    def __init__(self, flowController, pressureSensor, microphone, position, motors_controller, windowWidth=200, interval=10):
        self.flowController = flowController
        self.pressureSensor = pressureSensor
        self.microphone = microphone
        self.position = position
        self.motors_controller = motors_controller
        self.windowWidth = windowWidth
        self.flow_ref = linspace(0,0,self.windowWidth)
        self.x_ref = linspace(0,0,self.windowWidth)
        self.z_ref = linspace(0,0,self.windowWidth)
        self.alpha_ref = linspace(0,0,self.windowWidth)
        self.x = linspace(0,0,self.windowWidth)
        self.z = linspace(0,0,self.windowWidth)
        self.alpha = linspace(0,0,self.windowWidth)
        self.radius = linspace(0,0,self.windowWidth)
        self.theta = linspace(0,0,self.windowWidth)
        self.offset = linspace(0,0,self.windowWidth)
        self.mouth_pressure = linspace(0,0,self.windowWidth)
        self.volume_flow = linspace(0,0,self.windowWidth)
        self.mass_flow = linspace(0,0,self.windowWidth)
        self.temperature = linspace(0,0,self.windowWidth)
        self.frequency = linspace(0,0,self.windowWidth)
        self.times = linspace(0,0,self.windowWidth)
        self.t0 = time()

        self.interval = interval
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        
    def start(self):
        self.timer.start(self.interval)

    def update(self):
        self.flow_ref[:-1] = self.flow_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.flow_ref[-1] = self.flowController.values['set_point']
        self.x_ref[:-1] = self.x_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.x_ref[-1] = self.motors_controller.x_ref
        self.z_ref[:-1] = self.z_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.z_ref[-1] = self.motors_controller.z_ref
        self.alpha_ref[:-1] = self.alpha_ref[1:]                      # shift data in the temporal mean 1 sample left
        self.alpha_ref[-1] = self.motors_controller.alpha_ref
        self.x[:-1] = self.x[1:]                      # shift data in the temporal mean 1 sample left
        self.x[-1] = self.position.x
        self.z[:-1] = self.z[1:]                      # shift data in the temporal mean 1 sample left
        self.z[-1] = self.position.z
        self.alpha[:-1] = self.alpha[1:]                      # shift data in the temporal mean 1 sample left
        self.alpha[-1] = self.position.alpha
        self.radius[:-1] = self.radius[1:]                      # shift data in the temporal mean 1 sample left
        self.radius[-1] = self.position.r
        self.theta[:-1] = self.theta[1:]                      # shift data in the temporal mean 1 sample left
        self.theta[-1] = self.position.theta
        self.offset[:-1] = self.offset[1:]                      # shift data in the temporal mean 1 sample left
        self.offset[-1] = self.position.o
        self.mouth_pressure[:-1] = self.mouth_pressure[1:]                      # shift data in the temporal mean 1 sample left
        self.mouth_pressure[-1] = self.pressureSensor.values['pressure']
        self.volume_flow[:-1] = self.volume_flow[1:]                      # shift data in the temporal mean 1 sample left
        self.volume_flow[-1] = self.flowController.values['vol_flow']
        self.mass_flow[:-1] = self.mass_flow[1:]                      # shift data in the temporal mean 1 sample left
        self.mass_flow[-1] = self.flowController.values['mass_flow']
        self.temperature[:-1] = self.temperature[1:]                      # shift data in the temporal mean 1 sample left
        self.temperature[-1] = self.flowController.values['temperature']
        self.frequency[:-1] = self.frequency[1:]                      # shift data in the temporal mean 1 sample left
        self.frequency[-1] = self.microphone.pitch
        self.times[:-1] = self.times[1:]                      # shift data in the temporal mean 1 sample left
        self.times[-1] = time() - self.t0

class FlowSignalGenerator(threading.Thread):
    '''
    Esta clase se encarga de asignarle una referencia al flujo, permite conseguir rampas de distintas formas entre dos valores y la posibilidad de agregar vibratos (modificable amplitud y frecuencia)
    '''
    def __init__(self, callback, running, Fi=0, Ff=0, T=0, deformation=1, vibrato_amp=0, vibrato_freq=0):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.callback = callback
        self.running = running
        self.Fi = Fi
        self.Ff = Ff
        self.T = T
        self.deformation = deformation
        self.vibrato_amp = vibrato_amp
        self.vibrato_freq = vibrato_freq
        self.t0 = time()
        self.refresh_time = 0.01
        self.min_value = 0
        self.max_value = 50

    def run(self):
        val = self.Fi
        t = time() - self.t0

        while self.running.is_set():
            t = time() - self.t0
            if t < self.T:
                ramp = self.Fi + (self.Ff-self.Fi) * (t / self.T) ** self.deformation
                vibr = self.vibrato_amp * np.sin(t * 2*np.pi * self.vibrato_freq)
                val = ramp + vibr
            else:
                val = self.Ff + self.vibrato_amp * np.sin(t * 2*np.pi * self.vibrato_freq)
            val = self.saturate(val)
            #print(val)
            self.callback(val)
            sleep(self.refresh_time)

        self.callback(0)
        print("Flow signal thread killed")

    def move_to(self, next_state, T, deformation=1):
        self.Fi = self.Ff
        self.Ff = next_state.flow
        self.T = T
        self.deformation = deformation
        self.vibrato_amp = next_state.vibrato_amp
        self.vibrato_freq = next_state.vibrato_freq
        self.t0 = time()
        #print(self.Ff, self.vibrato_amp, self.vibrato_freq)

    def stop(self):
        self.T = 0
        self.Ff = 0
        self.vibrato_amp = 0

    def saturate(self, value):
        if value > self.max_value:
            value = self.max_value
        elif value < self.min_value:
            value = self.min_value
        return value

class Player(QtCore.QThread):
    '''
    Esta clase es la que interactúa con los controladores de motores, de flujo y de teclas. Es el equivalente al músico.
    '''
    finished_score = QtCore.pyqtSignal()
    finished_initial_positioning = QtCore.pyqtSignal()
    begin_phrase_action = QtCore.pyqtSignal(object)
    begin_finger_action = QtCore.pyqtSignal(object)

    def __init__(self, running, state, flow_controller, preasure_sensor, x_drive, z_drive, alpha_drive, microphone):
        QtCore.QThread.__init__(self)
        self.running = running
        self.state = state

        self.flow_controller = flow_controller
        self.flow_controller.flow_change_signal.connect(self.update_flow)

        self.preasure_sensor = preasure_sensor

        self.x_drive = x_drive
        self.z_drive = z_drive
        self.alpha_drive = alpha_drive

        self.microphone = microphone

        self.motors_event = threading.Event()
        self.motors_event.set()
        self.motors_controller = MotorsController(self.motors_event, state, x_drive, z_drive, alpha_drive)
        self.motors_controller.start()
        
        self.flowSignalEvent = threading.Event()
        self.flowSignalEvent.set()
        self.flow_reference_signal = FlowSignalGenerator(self.flow_controller.change_ref, self.flowSignalEvent, self.state.flow, self.state.flow, vibrato_amp=self.state.vibrato_amp, vibrato_freq=self.state.vibrato_freq, deformation=1)
        self.flow_reference_signal.start()
        
        self.recorder = Recorder(self.flow_controller, self.preasure_sensor, self.microphone, self.state, self.motors_controller)

        self.initial_position = None
        self.phrase_instructions = []
        self.finger_instructions = []
        self.next_state = State(0,0,0,0)
        self.next_state.homed()
        self.performing = threading.Event()
        self.playing = threading.Event()

    def update_flow(self, value):
        '''
        Actualiza el flujo de acuerdo a lo leido por el controlador de flujo
        '''
        self.state.flow = value

    def run(self):
        while self.running.is_set():
            self.performing.wait(timeout=1)
            if self.performing.is_set():
                self.play()
                self.performing.clear()
                self.playing.clear()
                #print('Score executed')
                self.finished_score.emit()
        
        self.motors_event.clear()
        self.flowSignalEvent.clear()
        print('Player thread killed')

    def move_to_state(self, desired_state, T=None, deformation=1, acc=50, dec=50, onlyCartesian=False, onlyFlow=False):
        '''
        Sirve para moverse a otro estado cambiando cada una de las dimensiones que se quiera cambiar: x, z, alpha (o equivalentemente r, theta y offset), flujo, amplitud del vibrato y frecuencia del vibrato. Además permite decidir cómo se quiere mover de un estado a otro.

        inputs:
        desired_state = el estado al que se quiere llegar. De la clase State
        T             = el tiempo en el que se quiere llegar. Si se deja en None, 
        deformation   = deformación de la rampa que sigue el flujo. Si 0<def<1 sigue una curva de segunda derivada negativa, si def==0 sigue una rampa y si 1<def sigue una curva de segunda derivada positiva
        acc           = aceleración de partida
        dec           = desaceleración de frenado
        onlyCartesian = bool si se quiere mover por trayectorias cartesianas
        onlyFlow      = bool si solo se quiere cambiar características del flujo
        '''
        if onlyFlow:
            if self.state.flow != desired_state.flow or self.state.vibrato_freq != desired_state.vibrato_freq or self.state.vibrato_amp != desired_state.vibrato_amp:
                self.flow_reference_signal.move_to(desired_state, T, deformation)
                self.state.vibrato_freq = desired_state.vibrato_freq
                self.state.vibrato_amp = desired_state.vibrato_amp
            return
        if desired_state.not_posible:
            print('Desired position not valid')
            return
        if onlyCartesian:
            self.motors_controller.move_cartesians_only(desired_state)
        else:
            T = self.motors_controller.move_to(desired_state, acc, dec, T)
            
            if self.state.flow != desired_state.flow or self.state.vibrato_freq != desired_state.vibrato_freq or self.state.vibrato_amp != desired_state.vibrato_amp:
                self.flow_reference_signal.move_to(desired_state, T, deformation)
                self.state.vibrato_freq = desired_state.vibrato_freq
                self.state.vibrato_amp = desired_state.vibrato_amp
            # self.state.change_state(desired_state)

    def move_to_next_state(self, T=None, deformation=1, speed=50, acc=5, dec=5, error=0.01, onlyCartesian=False):
        self.move_to_state(self.next_state, T=T, deformation=deformation, speed=speed, acc=acc, dec=dec, error=error, onlyCartesian=onlyCartesian)

    def play(self):
        '''
        Cuando se tiene una lista de instrucciones (self.initial_position, self.phrase_instructions y self.finger_instructions), esta función sirve para ejecutarlas en orden
        '''

        #print(self.initial_position)

        if self.initial_position:
            position = State(self.initial_position['r'], self.initial_position['theta'], self.initial_position['offset'], 0)
            self.move_to_state(position)
            paused = False
            while abs(self.state.r - self.initial_position['r']) > 0.2 or abs(self.state.theta - self.initial_position['theta']) > 0.2 or abs(self.state.o - self.initial_position['offset']) > 0.2:
                if not self.performing.is_set():
                    self.stop()
                    return
                if not self.playing.is_set():
                    if not paused:
                        self.stop()
                        paused = True
                else:
                    if paused:
                        self.move_to_state(position)
                        paused = False
            self.finished_initial_positioning.emit()

            all_actions = []
            ti = 0
            for instruction in self.phrase_instructions:
                all_actions.append({'ti': ti, 'type': 0, 'data': instruction})
                ti += instruction['time']
            ti = 0
            for instruction in self.finger_instructions:
                all_actions.append({'ti': ti, 'type': 1, 'data': instruction})
                ti += instruction['time']

            all_actions.sort(key=lambda x: x['ti'])

            t0 = time()
            next_t = 0
            last_ti = 0
            last_tf = 0
            phrase_actions_executed = 0
            finger_actions_executed = 0
            paused = False
            next_pos = None
            while len(all_actions) > 0:
                if not self.performing.is_set():
                    self.stop()
                    break
                if not self.playing.is_set():
                    if not paused:
                        paused = True
                        t_pause = time()
                        self.stop()
                    sleep(0.05)
                else:
                    if paused:
                        paused = False
                        t0 += time() - t_pause
                        if action['type'] == 0:
                            if next_pos:
                                position = State(next_pos['r'], next_pos['theta'], next_pos['offset'], next_pos['flow'], vibrato_freq=next_pos['vibrato_freq'], vibrato_amp=next_pos['vibrato_amp'])
                                self.move_to_state(position)
                                while abs(self.state.r - position.r) > 0.2 or abs(self.state.theta - position.theta) > 0.2 or abs(self.state.o - position.o) > 0.2:
                                    if not self.performing.is_set():
                                        self.stop()
                                        return
                                    if not self.playing.is_set():
                                        if not paused:
                                            self.stop()
                                            t_pause = time()
                                            paused = True
                                    else:
                                        if paused:
                                            self.move_to_state(position)
                                            paused = False
                                t0 = time() - next_t
                                next_pos = None
                    if time() - t0 >= next_t:
                        action = all_actions.pop(0)
                        if action['type'] == 0: # si es instrucción de la frase musical
                            self.begin_phrase_action.emit(phrase_actions_executed)
                            next_pos = action['data']
                            self.execute_phrase_action(action)
                            phrase_actions_executed += 1
                            if action['ti'] + action['data']['time'] > last_tf:
                                last_tf = action['data']['time']
                                last_ti = action['ti']
                            else:
                                last_tf -= (action['ti'] - last_ti)
                            if len(all_actions):
                                next_t = all_actions[0]['ti']
                        else: # si es instrucción de los dedos
                            self.begin_finger_action.emit(finger_actions_executed)
                            self.execute_fingers_action(action)
                            finger_actions_executed += 1
                            if action['ti'] + action['data']['time'] > last_tf:
                                last_tf = action['data']['time']
                                last_ti = action['ti']
                            else:
                                last_tf -= (action['ti'] - last_ti)
                            if len(all_actions):
                                next_t = all_actions[0]['ti']
                    else:
                        sleep(0.05)
            t0 = time()
            paused = False
            t_pause = 0
            print('Last step')
            while time() - t0 < last_tf and not paused:
                if not self.performing.is_set():
                    self.stop()
                    return
                if not self.playing.is_set():
                    if not paused:
                        self.stop()
                        t_pause = time()
                        paused = True
                else:
                    if paused:
                        self.move_to_state(position)
                        t0 += time() - t_pause
                        paused = False
            print('Done')
        # print(self.phrase_instructions)
        # print(self.finger_instructions)


    def execute_phrase_action(self, action):
        '''
        Ejecuta una action de la frase musical (posición + flujo)
        '''
        if action['data']['move']:
            desired_state = State(action['data']['r'], action['data']['theta'], action['data']['offset'], action['data']['flow'], vibrato_freq=action['data']['vibrato_freq'], vibrato_amp=action['data']['vibrato_amp'])

            self.move_to_state(desired_state, T=action['data']['time'], deformation=action['data']['deformation'], acc=action['data']['acceleration'], dec=action['data']['deceleration'])

    def execute_fingers_action(self, action):
        '''
        Ejecuta una acción de los dedos
        '''
        pass

    def stop(self):
        self.motors_controller.stop()
        self.flow_reference_signal.stop()

    def moving(self):
        moving_x = self.motors_controller.x_driver.moving_cw or self.motors_controller.x_driver.moving_ccw
        moving_z = self.motors_controller.z_driver.moving_cw or self.motors_controller.z_driver.moving_ccw
        moving_alpha = self.motors_controller.alpha_driver.moving_cw or self.motors_controller.alpha_driver.moving_ccw

        if moving_x or moving_z or moving_alpha:
            return True
        else:
            return False

    def auto_home(self, x=True, z=True, alpha=False):
        if x:
            #self.motors_controller.x_driver.request_write_ccw_find_home(programmed_speed=1000, acceleration=500, deceleration=500)
            self.motors_controller.x_driver.request_write_ccw_find_home_to_limit()
            pass
        if z:
            #self.motors_controller.z_driver.request_write_ccw_find_home(programmed_speed=1000, acceleration=500, deceleration=500)
            self.motors_controller.z_driver.request_write_ccw_find_home_to_limit()
            pass
        if alpha:
            self.motors_controller.alpha_driver.request_write_ccw_find_home_to_limit()
            
    def finish_autohome(self):
        #self.motors_controller.x_driver.request_write_set_starting_speed(1)
        #self.motors_controller.z_driver.request_write_set_starting_speed(1)
        #self.motors_controller.alpha_driver.request_write_set_starting_speed(1)

        self.motors_controller.homed()