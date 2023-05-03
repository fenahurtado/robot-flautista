from asyncore import read
from turtle import color
from numpy import *
import json
import os
import matplotlib.pyplot as plt
plt.rcParams['text.usetex'] = True

class State:
    '''
    Esta clase es de utilidad para pasarse entre las variables controlables y las variables del sistema.
    '''
    def __init__(self, r, theta, o, flow, vibrato_freq=0, vibrato_amp=0):
        self._r = r
        self._theta = theta
        self._o = o
        self.not_posible = False
        try:
            x, z, alpha = get_x_z_alpha(r, theta, o)
        except:
            x, z, alpha = 0, 0, 0
            self.not_posible = True
        self._x = x
        self._z = z
        self._alpha = alpha
        self.flow = flow
        self.vibrato_freq = vibrato_freq
        self.vibrato_amp = vibrato_amp
    
    @property
    def r(self):
        return round(self._r,2)
    
    @r.setter
    def r(self, other):
        self._r = other
        try:
            self._x, self._z, self._alpha = get_x_z_alpha(self._r, self._theta, self._o)
            self.not_posible = False
        except:
            self.not_posible = True
            print('Non real number')
    
    @property
    def theta(self):
        return round(self._theta,2)
    
    @theta.setter
    def theta(self, other):
        self._theta = other
        try:
            self._x, self._z, self._alpha = get_x_z_alpha(self._r, self._theta, self._o)
            self.not_posible = False
        except:
            self.not_posible = True
            print('Non real number')
    
    @property
    def o(self):
        return round(self._o,2)
    
    @o.setter
    def o(self, other):
        self._o = other
        try:
            self._x, self._z, self._alpha = get_x_z_alpha(self._r, self._theta, self._o)
            self.not_posible = False
        except:
            self.not_posible = True
            print('Non real number')
    
    @property
    def x(self):
        return round(self._x,2)
    
    @x.setter
    def x(self, other):
        self._x = other
        self._r, self._theta, self._o = get_r_theta_o(self._x, self._z, self._alpha)
    
    @property
    def z(self):
        return round(self._z, 2)
    
    @z.setter
    def z(self, other):
        self._z = other
        self._r, self._theta, self._o = get_r_theta_o(self._x, self._z, self._alpha)
    
    @property
    def alpha(self):
        return round(self._alpha,2)
    
    @alpha.setter
    def alpha(self, other):
        self._alpha = other
        self._r, self._theta, self._o = get_r_theta_o(self._x, self._z, self._alpha)
    
    def __str__(self):
        return f'r: {self.r}, theta: {self.theta}, offset: {self.o}, flow: {self.flow}'
    
    def cart_coords(self):
        return self.x, self.z, self.alpha
    
    def flute_coords(self):
        return self.r, self.theta, self.o

    def homed(self):
        self.x = 0
        self.z = 0
        self.alpha = 0
    
    def change_state(self, other):
        self.r = other.r
        self.theta = other.theta
        self.o = other.o
        self.flow = other.flow
        self.vibrato_amp = other.vibrato_amp
        self.vibrato_freq = other.vibrato_freq

    def is_too_close(self, other, thr_x=0.3, thr_z=0.3, thr_alpha=0.5):
        if abs(other.x - self.x) < thr_x and abs(other.z - self.z) < thr_z and abs(other.x - self.x) < thr_alpha:
            return True
        return False

        
def read_variables():
    dir = os.path.dirname(os.path.realpath(__file__)) + '/settings.json'
    with open(dir) as json_file:
        DATA = json.load(json_file)
    return DATA

DATA = read_variables()
DATA_dir = dir = os.path.dirname(os.path.realpath(__file__)) + '\settings.json'

def save_variables():
    global DATA, DATA_dir
    with open(DATA_dir, 'w') as json_file:
        json.dump(DATA, json_file, indent=4, sort_keys=True)

def get_pos_punta(x,z,alpha, offset_x=0, offset_z=0):
    global DATA
    #alpha = alpha * pi / 180
    x2 = x  + DATA["physical_constants"]["dm"] + DATA["physical_constants"]["dp"]*cos(alpha) - DATA["physical_constants"]["dq"]*sin(alpha)
    z2 = z + DATA["physical_constants"]["dp"]*sin(alpha) + DATA["physical_constants"]["dq"]*cos(alpha)
    return x2, z2

def get_r_theta_o(x,z,alpha):
    global DATA
    alpha = alpha * pi / 180
    x2, z2 = get_pos_punta(x,z,alpha)
    r = sqrt((x2 - DATA["flute_position"]["X_F"])**2 + (z2 - DATA["flute_position"]["Z_F"])**2)
    theta = alpha + DATA["flute_position"]["alpha_flauta"]
    if theta > 2*pi:
        theta -= 2*pi
    if theta < 0:
        theta += 2*pi
    o = (x2 - DATA["flute_position"]["X_F"])*tan(alpha) - (z2 - DATA["flute_position"]["Z_F"])
    return r, theta*180/pi, o

def get_x_z_alpha(r, theta, o):
    global DATA
    theta = theta * pi / 180
    alpha = theta - DATA["flute_position"]["alpha_flauta"]
    a = (1 + tan(alpha)**2)
    b = (-2*o*tan(alpha))
    c = (o**2 - r**2)
    if b**2 - 4*a*c < 0:
        raise Exception('Non real number')
    x2 = (-b - sqrt(b**2 - 4*a*c))/(2*a) + DATA["flute_position"]["X_F"]
    z2 = - o + (x2-DATA["flute_position"]["X_F"])*tan(alpha) + DATA["flute_position"]["Z_F"]
    x = x2 - DATA["physical_constants"]["dm"] - DATA["physical_constants"]["dp"]*cos(alpha) + DATA["physical_constants"]["dq"]*sin(alpha)
    z = z2 - DATA["physical_constants"]["dp"]*sin(alpha) - DATA["physical_constants"]["dq"]*cos(alpha)
    return x, z, alpha*180/pi


change_to_joint_space = vectorize(get_x_z_alpha)
change_to_task_space = vectorize(get_r_theta_o)

def x_mm_to_units(mm):
    return int(round(mm * 4000 / 8 , 0))

def alpha_angle_to_units(angle):
    return int(round(angle * 4000 / 360, 0))
    
mm2units = vectorize(x_mm_to_units)
angle2units = vectorize(alpha_angle_to_units)

if __name__ == "__main__":
    ri, thetai, oi = 22.67, 45.0, 19.5
    rf, thetaf, of = 16.0, 65.0, 0.0
    T = 1
    
    print(get_r_theta_o(*get_x_z_alpha(31,63,13)))
    print(get_x_z_alpha(*get_r_theta_o(66,87,18)))
    #plan_route_min_error(ri, thetai, oi, rf, thetaf, of, error=0.05, plot=True)

    # x_points, z_points, alpha_points, d = plan_route_min_error(ri, thetai, oi, rf, thetaf, of, error=0.05, plot=True)
    # fig, ax = plt.subplots(figsize=(6,6))
    # ax.plot(x_points, z_points, color='b')
    # plt.show()