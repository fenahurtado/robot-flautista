import sys
sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
from utils.motor_route import get_route_positions, plan_speed_curve, plan_temps_according_to_speed, x_mm_to_units, z_mm_to_units, alpha_angle_to_units
from utils.cinematica import State, get_r_theta_o
import numpy as np
from matplotlib import pyplot as plt

def new_plan_route(x_points, z_points, alpha_points, temps, aprox=True):
    points = {'x': [], 'z': [], 'alpha': [], 'r': [], 'theta': [], 'o': [], 't': []}

    for i in range(len(x_points) - 1):
        r, theta, o = get_r_theta_o(x_points[i], z_points[i], alpha_points[i])
        x = x_mm_to_units(x_points[i], aprox=aprox)            
        z = z_mm_to_units(z_points[i], aprox=aprox)
        alpha = alpha_angle_to_units(alpha_points[i], aprox=aprox)
        t = temps[i]

        points['x'].append(x)
        points['z'].append(z)
        points['alpha'].append(alpha)
        points['r'].append(r)
        points['theta'].append(theta)
        points['o'].append(o)
        points['t'].append(t)

    return points

initial_state = State(5.5, 45, 0, 0)
final_state   = State(9.5, 45, 0, 0)
divisions     = 300
acc           = 1
dec           = 1
T             = 5
aprox         = False

x_points, z_points, alpha_points, d = get_route_positions(*initial_state.cart_coords(), *final_state.cart_coords(), divisions=divisions, plot=False)

vel, t_acc, t_dec = plan_speed_curve(d[-1], acc, dec, T)
temps = plan_temps_according_to_speed(d, vel, t_acc, t_dec, acc, dec)
route = new_plan_route(x_points, z_points, alpha_points, temps, aprox=aprox)
route['x'].append(x_mm_to_units(final_state.x, aprox=aprox))
route['z'].append(z_mm_to_units(final_state.z, aprox=aprox))
route['alpha'].append(alpha_angle_to_units(final_state.alpha, aprox=aprox))
route['theta'].append(final_state.theta)
route['o'].append(final_state.o)
route['r'].append(final_state.r)
route['t'].append(T)

plt.plot(route['t'], route['r'])
plt.show()