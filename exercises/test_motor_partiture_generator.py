import json
import os
from datetime import datetime
import sys
sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
from utils.motor_route import max_dist_rec

notas = []

init_pos = {
        "offset": 0,
        "r": 46.35,
        "theta": 45,
        "type": 0
    }

phrase = []

step_1 = 15
step_2_multiplier = 2
Tslow = 0.5
dT = 0.05

step_2 = step_1*step_2_multiplier
iteracion = 0
acc = 9999
dec = 9999

while max_dist_rec(acc, dec, Tslow - dT*iteracion) > step_1 and max_dist_rec(acc, dec, step_2_multiplier*(Tslow - dT*iteracion)) > step_2:
    up_corto = {
        "acceleration": acc,
        "deceleration": dec,
        "deformation": 1,
        "flow": 0,
        "jerk": 0,
        "move": 1,
        "offset": 0,
        "r": 46.35-step_1,
        "theta": 45,
        "time": Tslow-dT*iteracion,
        "type": 1,
        "vibrato_amp": 0,
        "vibrato_freq": 0
    }
    stay_up = {
        "acceleration": acc,
        "deceleration": dec,
        "deformation": 1,
        "flow": 0,
        "jerk": 0,
        "move": 1,
        "offset": 0,
        "r": 46.35-step_1,
        "theta": 45,
        "time": 0.5,
        "type": 1,
        "vibrato_amp": 0,
        "vibrato_freq": 0
    }
    down_corto = {
        "acceleration": acc,
        "deceleration": dec,
        "deformation": 1,
        "flow": 0,
        "jerk": 0,
        "move": 1,
        "offset": 0,
        "r": 46.35,
        "theta": 45,
        "time": Tslow-dT*iteracion,
        "type": 1,
        "vibrato_amp": 0,
        "vibrato_freq": 0
    }
    stay_down = {
        "acceleration": acc,
        "deceleration": dec,
        "deformation": 1,
        "flow": 0,
        "jerk": 0,
        "move": 1,
        "offset": 0,
        "r": 46.35,
        "theta": 45,
        "time": 0.5,
        "type": 1,
        "vibrato_amp": 0,
        "vibrato_freq": 0
    }
    up_largo = {
        "acceleration": acc,
        "deceleration": dec,
        "deformation": 1,
        "flow": 0,
        "jerk": 0,
        "move": 1,
        "offset": 0,
        "r": 46.35-step_2,
        "theta": 45,
        "time": step_2_multiplier*(Tslow-dT*iteracion),
        "type": 1,
        "vibrato_amp": 0,
        "vibrato_freq": 0
    }
    stay_up2 = {
        "acceleration": acc,
        "deceleration": dec,
        "deformation": 1,
        "flow": 0,
        "jerk": 0,
        "move": 1,
        "offset": 0,
        "r": 46.35-step_2,
        "theta": 45,
        "time": 0.5,
        "type": 1,
        "vibrato_amp": 0,
        "vibrato_freq": 0
    }
    down_largo = {
        "acceleration": acc,
        "deceleration": dec,
        "deformation": 1,
        "flow": 0,
        "jerk": 0,
        "move": 1,
        "offset": 0,
        "r": 46.35,
        "theta": 45,
        "time": step_2_multiplier*(Tslow-dT*iteracion),
        "type": 1,
        "vibrato_amp": 0,
        "vibrato_freq": 0
    }
    stay_down2 = {
        "acceleration": acc,
        "deceleration": dec,
        "deformation": 1,
        "flow": 0,
        "jerk": 0,
        "move": 1,
        "offset": 0,
        "r": 46.35,
        "theta": 45,
        "time": 0.5,
        "type": 1,
        "vibrato_amp": 0,
        "vibrato_freq": 0
    }
    phrase += [up_corto, stay_up, down_corto, stay_down, up_largo, stay_up2, down_largo, stay_down2]
    iteracion += 1

filename = '/home/fernando/Dropbox/UC/Magister/robot-flautista/exercises/test_motor_2.json'

data = {'init_pos': init_pos, 'phrase': phrase, 'fingers': notas, 'timestamp': datetime.now().strftime("%d/%m/%Y %H:%M:%S")}
with open(filename, 'w') as file:
    json.dump(data, file, indent=4, sort_keys=True)