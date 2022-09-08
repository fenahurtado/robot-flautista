import json
import os
from datetime import datetime

t_negra = 0.25

correccion_radio  = 0
correccion_theta  = 0
correccion_offset = 0

filename = '/home/fernando/Dropbox/UC/Magister/robot-flautista/examples/escala_1.json'

fingers = [
        # {
        #     "note": "D3",
        #     "time": 2
        # },
        # {
        #     "note": "E3",
        #     "time": 1
        # },
        # {
        #     "note": "F#3",
        #     "time": 1
        # },
        # {
        #     "note": "G3",
        #     "time": 1
        # },
        # {
        #     "note": "A3",
        #     "time": 1
        # },
        # {
        #     "note": "B3",
        #     "time": 1
        # },
        # {
        #     "note": "C#4",
        #     "time": 1
        # },
        # {
        #     "note": "D4",
        #     "time": 2
        # },
        # {
        #     "note": "E4",
        #     "time": 1
        # },
        # {
        #     "note": "F#4",
        #     "time": 1
        # },
        # {
        #     "note": "G4",
        #     "time": 1
        # },
        # {
        #     "note": "A4",
        #     "time": 1
        # },
        # {
        #     "note": "B4",
        #     "time": 1
        # },
        # {
        #     "note": "C#5",
        #     "time": 1
        # },
        # {
        #     "note": "D5",
        #     "time": 2
        # },
        # {
        #     "note": "C#5",
        #     "time": 1
        # },
        # {
        #     "note": "B4",
        #     "time": 1
        # },
        # {
        #     "note": "A4",
        #     "time": 1
        # },
        # {
        #     "note": "G4",
        #     "time": 1
        # },
        # {
        #     "note": "F#4",
        #     "time": 1
        # },
        # {
        #     "note": "E4",
        #     "time": 1
        # },
        # {
        #     "note": "D4",
        #     "time": 2
        # },
        # {
        #     "note": "C#4",
        #     "time": 1
        # },
        # {
        #     "note": "B3",
        #     "time": 1
        # },
        # {
        #     "note": "A3",
        #     "time": 1
        # },
        # {
        #     "note": "G3",
        #     "time": 1
        # },
        # {
        #     "note": "F#3",
        #     "time": 1
        # },
        # {
        #     "note": "E3",
        #     "time": 1
        # },
        # {
        #     "note": "D3",
        #     "time": 2
        # }
]

init_pos = {
        "offset": 0.5,
        "r": 8.5,
        "theta": 40.97
    }

phrase  = [
        {
            "acceleration": 99.0,
            "deceleration": 99.0,
            "deformation": 1,
            "flow": 0, #9.5,
            "jerk": 0,
            "move": 1,
            "offset": 0.5,
            "r": 8.5,
            "theta": 40.97,
            "time": 0.5,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "move": 0,
            "time": 0.5
        },
        {
            "acceleration": 99.0,
            "deceleration": 99.0,
            "deformation": 1,
            "flow": 0, #12.0,
            "jerk": 0,
            "move": 1,
            "offset": -0.0,
            "r": 7.51,
            "theta": 44.97,
            "time": 7.5,
            "vibrato_amp": 0.0,
            "vibrato_freq": 0.0
        },
        {
            "move": 0,
            "time": 1.0
        },
        {
            "acceleration": 99.0,
            "deceleration": 99.0,
            "deformation": 1,
            "flow": 0, #30.0,
            "jerk": 0,
            "move": 1,
            "offset": 1,
            "r": 6.51,
            "theta": 48.0,
            "time": 7.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "move": 0,
            "time": 1.0
        },
        {
            "acceleration": 99.0,
            "deceleration": 99.0,
            "deformation": 1,
            "flow": 0, #12.0,
            "jerk": 0,
            "move": 1,
            "offset": -0.0,
            "r": 7.51,
            "theta": 44.97,
            "time": 7,
            "vibrato_amp": 0.0,
            "vibrato_freq": 0.0
        },
        {
            "move": 0,
            "time": 1.0
        },
        {
            "acceleration": 99.0,
            "deceleration": 99.0,
            "deformation": 1,
            "flow": 0, #9.5,
            "jerk": 0,
            "move": 1,
            "offset": 0.5,
            "r": 8.5,
            "theta": 40.97,
            "time": 7,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "move": 0,
            "time": 1
        },
        {
            "acceleration": 99.0,
            "deceleration": 99.0,
            "deformation": 1,
            "flow": 0.0,
            "jerk": 0,
            "move": 1,
            "offset": 0.5,
            "r": 8.5,
            "theta": 40.97,
            "time": 0.5,
            "vibrato_amp": 0.1,
            "vibrato_freq": 0.1
        }
]

last_flow = 0
last_offset = init_pos['offset']
last_r = init_pos['r']
last_theta = init_pos['theta']
last_vibrato_amp = 0
last_vibrato_freq = 0

init_pos['type'] = 0
init_pos['r'] += correccion_radio
init_pos['theta'] += correccion_theta
init_pos['offset'] += correccion_offset
for i in fingers:
    i['type'] = 2
    i['time'] = i['time']*t_negra
for i in phrase:
    i['type'] = 1
    i['time'] = i['time']*t_negra
    if i['move']:
        i['r'] += correccion_radio
        i['theta'] += correccion_theta
        i['offset'] += correccion_offset
        last_flow = i['flow']
        last_offset = i['offset']
        last_r = i['r']
        last_theta = i['theta']
        last_vibrato_amp = i['vibrato_amp']
        last_vibrato_freq = i['vibrato_freq']
    else:
        i["acceleration"] = 99.0
        i["deceleration"] = 99.0
        i["deformation"] = 1
        i["flow"] = last_flow
        i["jerk"] = 0
        i["offset"] = last_offset
        i["r"] = last_r
        i["theta"] = last_theta
        i["vibrato_amp"] = last_vibrato_amp
        i["vibrato_freq"] = last_vibrato_freq

base_path = '/home/fernando/Dropbox/UC/Magister/robot-flautista/view_control'
data = {'init_pos': init_pos, 'phrase': phrase, 'fingers': fingers, 'timestamp': datetime.now().strftime("%d/%m/%Y %H:%M:%S")}
with open(filename, 'w') as file:
    json.dump(data, file, indent=4, sort_keys=True)
if 'recent_saves.txt' in os.listdir(base_path):
    with open(base_path + '/recent_saves.txt', 'r+') as file:
        content = file.read()
        file.seek(0, 0)
        file.write(filename + '\n' + content)
else:
    with open(base_path + '/recent_saves.txt', 'w') as file:
        file.write(filename)