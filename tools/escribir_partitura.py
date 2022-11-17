import json
import os
from datetime import datetime

t_negra = 0.45

correccion_radio  = 0
correccion_theta  = 0
correccion_offset = 0

filename = '/home/fernando/Dropbox/UC/Magister/robot-flautista/examples/escala_1.json'

fingers = [
        {
            "note": "D3",
            "time": 3.0,
            "type": 2
        },
        {
            "note": "E3",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "F#3",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "G3",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "A3",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "B3",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "C#4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "D4",
            "time": 2.0,
            "type": 2
        },
        {
            "note": "E4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "F#4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "G4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "A4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "B4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "C#5",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "D5",
            "time": 2.0,
            "type": 2
        },
        {
            "note": "C#5",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "B4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "A4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "G4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "F#4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "E4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "D4",
            "time": 2.0,
            "type": 2
        },
        {
            "note": "C#4",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "B3",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "A3",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "G3",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "F#3",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "E3",
            "time": 1.0,
            "type": 2
        },
        {
            "note": "D3",
            "time": 2.0,
            "type": 2
        }
]

init_pos = {
        "offset": 0.0,
        "r": 10.5,
        "theta": 45.0,
        "type": 0
    }

phrase  = [

        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 7.0,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 8.0,
            "theta": 45.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 0,
            "deceleration": 0,
            "deformation": 1,
            "flow": 7.0,
            "jerk": 0,
            "move": 0,
            "offset": -1.0,
            "r": 8.0,
            "theta": 45.0,
            "time": 1.5,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 8.6,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 7.5,
            "theta": 45.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 8.6,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 7.5,
            "theta": 45.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 9.2,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 7.0,
            "theta": 45.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 10.0,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 6.8,
            "theta": 45.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 11.0,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 6.3,
            "theta": 45.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 11.8,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 5.3,
            "theta": 49,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 12.3,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 5.3,
            "theta": 49.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 0,
            "deceleration": 0,
            "deformation": 1,
            "flow": 12.3,
            "jerk": 0,
            "move": 0,
            "offset": -1.0,
            "r": 5.3,
            "theta": 49.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 13.3,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 5.3,
            "theta": 49.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 14.3,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 5.4,
            "theta": 47.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 14.8,
            "jerk": 0,
            "move": 1,
            "offset": -1.2,
            "r": 5.4,
            "theta": 48.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 15.8,
            "jerk": 0,
            "move": 1,
            "offset": -1.4,
            "r": 5.3,
            "theta": 49.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 16.8,
            "jerk": 0,
            "move": 1,
            "offset": -1.5,
            "r": 5.2,
            "theta": 50.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 18.5,
            "jerk": 0,
            "move": 1,
            "offset": -1.5,
            "r": 5.1,
            "theta": 52.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 19.5,
            "jerk": 0,
            "move": 1,
            "offset": -1.5,
            "r": 5.0,
            "theta": 53.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 0,
            "deceleration": 0,
            "deformation": 1,
            "flow": 19.5,
            "jerk": 0,
            "move": 0,
            "offset": -1.5,
            "r": 5.0,
            "theta": 53.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 18.5,
            "jerk": 0,
            "move": 1,
            "offset": -1.5,
            "r": 5.1,
            "theta": 52.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 16.8,
            "jerk": 0,
            "move": 1,
            "offset": -1.5,
            "r": 5.2,
            "theta": 50.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 15.8,
            "jerk": 0,
            "move": 1,
            "offset": -1.4,
            "r": 5.3,
            "theta": 49.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 14.8,
            "jerk": 0,
            "move": 1,
            "offset": -1.2,
            "r": 5.4,
            "theta": 48.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 14.3,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 5.4,
            "theta": 50.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 13.3,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 5.3,
            "theta": 49.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 12.3,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 5.3,
            "theta": 49.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 0,
            "deceleration": 0,
            "deformation": 1,
            "flow": 12.3,
            "jerk": 0,
            "move": 0,
            "offset": -1.0,
            "r": 5.3,
            "theta": 49.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 11.8,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 5.3,
            "theta": 49,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 11.0,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 6.3,
            "theta": 45.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 10.0,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 6.8,
            "theta": 45.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 9.2,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 7.0,
            "theta": 45.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 8.6,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 7.5,
            "theta": 45.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 8.6,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 7.5,
            "theta": 45.5,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 7.0,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 8.0,
            "theta": 45.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 0,
            "deceleration": 0,
            "deformation": 1,
            "flow": 7.0,
            "jerk": 0,
            "move": 0,
            "offset": -1.0,
            "r": 8.0,
            "theta": 45.0,
            "time": 1.5,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        },
        {
            "acceleration": 90.0,
            "deceleration": 90.0,
            "deformation": 1,
            "flow": 0.0,
            "jerk": 0,
            "move": 1,
            "offset": -1.0,
            "r": 10.5,
            "theta": 45.0,
            "time": 1.0,
            "type": 1,
            "vibrato_amp": 0.01,
            "vibrato_freq": 0.01
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
        i["acceleration"] = 153.0
        i["deceleration"] = 153.0
    else:
        i["acceleration"] = 153.0
        i["deceleration"] = 153.0
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