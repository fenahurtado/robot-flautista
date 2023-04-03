from datetime import datetime
import json
import os

import numpy as np

if __name__ == "__main__":
    import sys
    from pathlib import Path # if you haven't already done so
    file = Path(__file__).resolve()
    parent, root = file.parent, file.parents[1]
    sys.path.append(str(root))

    # Additionally remove the current file's directory from sys.path
    try:
        sys.path.remove(str(parent))
    except ValueError: # Already removed
        pass

from utils.cinematica import State
from utils.motor_route import get_route_positions

MAX_ACC = 153.0
base_path = os.path.dirname(os.path.realpath(__file__))

notes = ['D3','D#3','E3','F3','F#3','G3','G#3','A3','A#3','B3','C4','C#4','D4','D#4','E4','F4','F#4','G4','G#4','A4','A#4','B4','C5','C#5','D5','D#5','E5','F5','F#5','G5','G#5','A5','C6']

with open('tools/look_up_table.json') as json_file:
    note_position = json.load(json_file)

def update_note_position():
    global note_position
    with open('tools/look_up_table.json') as json_file:
        note_position = json.load(json_file)

def dist_notas(note_1, note_2):
    global note_position
    note_1_pos = note_position[note_1]
    state1 = State(note_1_pos['r'], note_1_pos['theta'], note_1_pos['offset'], note_1_pos['flow'])
    note_2_pos = note_position[note_2]
    state2 = State(note_2_pos['r'], note_2_pos['theta'], note_2_pos['offset'], note_2_pos['flow'])
    x_points, z_points, alpha_points, d = get_route_positions(*state1.cart_coords(), *state2.cart_coords(), divisions=100, plot=False)
    return d[-1]

def min_time(note_1, note_2, acc):
    distance = dist_notas(note_1, note_2)
    return np.sqrt(4*distance/acc)

def min_time_pos(r_1, theta_1, o_1, r_2, theta_2, o_2, acc=153):
    state1 = State(r_1, theta_1, o_1, 0)
    state2 = State(r_2, theta_2, o_2, 0)
    x_points, z_points, alpha_points, d = get_route_positions(*state1.cart_coords(), *state2.cart_coords(), divisions=100, plot=False)
    dist = d[-1]
    return np.sqrt(4*dist/acc)

def change_speed(path, scale, new_path, notes_only=False):
    global MAX_ACC, base_path

    with open(path) as json_file:
        input_score = json.load(json_file)
    
    init_pos = input_score['init_pos']
    fingers = input_score['fingers']
    phrase = input_score['phrase']

    last_offset = init_pos['offset']
    last_r = init_pos['r']
    last_theta = init_pos['theta']

    for i in fingers:
        i['time'] = i['time']*scale
        i["acceleration"] = MAX_ACC
        i["deceleration"] = MAX_ACC
    if not notes_only:
        for i in phrase:
            i['time'] = i['time']*scale
            if i['move']:
                min_t_possible = min_time_pos(last_r, last_theta, last_offset, i['r'], i['theta'], i['offset'], acc=MAX_ACC)
                if i['time'] < min_t_possible:
                    raise Exception
    else:
        phrase = []

        # if i['move']:
        #     last_flow = i['flow']
        #     last_offset = i['offset']
        #     last_r = i['r']
        #     last_theta = i['theta']
        #     last_vibrato_amp = i['vibrato_amp']
        #     last_vibrato_freq = i['vibrato_freq']
        #     i["acceleration"] = MAX_ACC
        #     i["deceleration"] = MAX_ACC
        # else:
        #     i["acceleration"] = MAX_ACC
        #     i["deceleration"] = MAX_ACC
        #     i["deformation"] = 1
        #     i["flow"] = last_flow
        #     i["jerk"] = 0
        #     i["offset"] = last_offset
        #     i["r"] = last_r
        #     i["theta"] = last_theta
        #     i["vibrato_amp"] = last_vibrato_amp
        #     i["vibrato_freq"] = last_vibrato_freq

    data = {'init_pos': init_pos, 'phrase': phrase, 'fingers': fingers, 'timestamp': datetime.now().strftime("%d/%m/%Y %H:%M:%S")}
    with open(new_path, 'w') as file:
        json.dump(data, file, indent=4, sort_keys=True)
    if 'recent_saves.txt' in os.listdir(base_path):
        with open(base_path + '/recent_saves.txt', 'r+') as file:
            content = file.read()
            file.seek(0, 0)
            file.write(new_path + '\n' + content)
    else:
        with open(base_path + '/recent_saves.txt', 'w') as file:
            file.write(new_path)
            
def generate_states_from_notes(path, new_path, acc=100, selection=[], min_time_change=0):
    global MAX_ACC, base_path, note_position
    
    with open(path) as json_file:
        input_score = json.load(json_file)
    
    init_pos = input_score['init_pos']
    fingers = input_score['fingers']
    phrase = []

    if len(selection) == 0:
        selection = [True for i in fingers]
    
    t_acumul = 0
    last_ref = None
    for act in range(len(fingers)):
        note = fingers[act]['note']
        t = fingers[act]['time']
        if act == 0:
            init_pos['r'] = note_position[note]['r']
            init_pos['theta'] = note_position[note]['theta']
            init_pos['offset'] = note_position[note]['offset']
            phrase.append({
                "acceleration": MAX_ACC,
                "deceleration": MAX_ACC,
                "deformation": 0,
                "flow": note_position[note]['flow'],
                "jerk": 0,
                "move": 1,
                "offset": note_position[note]['offset'],
                "r": note_position[note]['r'],
                "theta": note_position[note]['theta'],
                "time": 0.1,
                "type": 1,
                "vibrato_amp": 0,
                "vibrato_freq": 0
            })
            t_acumul += t-0.1
            last_ref = note
        else:
            if not selection[act]:
                t_acumul += t
            else:
                t_min = min_time(last_ref, note, acc)
                t_change = max(t_min, min_time_change)
                if t_acumul > t_change:
                    phrase.append({
                        "acceleration": MAX_ACC,
                        "deceleration": MAX_ACC,
                        "deformation": 0,
                        "flow": note_position[last_ref]['flow'],
                        "jerk": 0,
                        "move": 0,
                        "offset": note_position[last_ref]['offset'],
                        "r": note_position[last_ref]['r'],
                        "theta": note_position[last_ref]['theta'],
                        "time": t_acumul-t_change,
                        "type": 1,
                        "vibrato_amp": 0,
                        "vibrato_freq": 0
                    })
                    phrase.append({
                        "acceleration": MAX_ACC,
                        "deceleration": MAX_ACC,
                        "deformation": 0,
                        "flow": note_position[note]['flow'],
                        "jerk": 0,
                        "move": 1,
                        "offset": note_position[note]['offset'],
                        "r": note_position[note]['r'],
                        "theta": note_position[note]['theta'],
                        "time": t_change,
                        "type": 1,
                        "vibrato_amp": 0,
                        "vibrato_freq": 0
                    })
                else:
                    print(f'Not enough time to change from {last_ref} to {note}')
                    raise Exception
                t_acumul = t
                last_ref = note
    if t_acumul:
        phrase.append({
            "acceleration": MAX_ACC,
            "deceleration": MAX_ACC,
            "deformation": 0,
            "flow": note_position[last_ref]['flow'],
            "jerk": 0,
            "move": 0,
            "offset": note_position[last_ref]['offset'],
            "r": note_position[last_ref]['r'],
            "theta": note_position[last_ref]['theta'],
            "time": t_acumul,
            "type": 1,
            "vibrato_amp": 0,
            "vibrato_freq": 0
        })
    
    data = {'init_pos': init_pos, 'phrase': phrase, 'fingers': fingers, 'timestamp': datetime.now().strftime("%d/%m/%Y %H:%M:%S")}
    with open(new_path, 'w') as file:
        json.dump(data, file, indent=4, sort_keys=True)
    if 'recent_saves.txt' in os.listdir(base_path):
        with open(base_path + '/recent_saves.txt', 'r+') as file:
            content = file.read()
            file.seek(0, 0)
            file.write(new_path + '\n' + content)
    else:
        with open(base_path + '/recent_saves.txt', 'w') as file:
            file.write(new_path)

def add_vibrato(path, vibrato_amp, vibrato_freq, new_path):
    global MAX_ACC, base_path
    
    with open(path) as json_file:
        input_score = json.load(json_file)
    
    init_pos = input_score['init_pos']
    fingers = input_score['fingers']
    phrase = input_score['phrase']

    for i in phrase:
        i['vibrato_amp'] += vibrato_amp
        i['vibrato_freq'] += vibrato_freq

    data = {'init_pos': init_pos, 'phrase': phrase, 'fingers': fingers, 'timestamp': datetime.now().strftime("%d/%m/%Y %H:%M:%S")}
    with open(new_path, 'w') as file:
        json.dump(data, file, indent=4, sort_keys=True)
    if 'recent_saves.txt' in os.listdir(base_path):
        with open(base_path + '/recent_saves.txt', 'r+') as file:
            content = file.read()
            file.seek(0, 0)
            file.write(new_path + '\n' + content)
    else:
        with open(base_path + '/recent_saves.txt', 'w') as file:
            file.write(new_path)

def change_all_acc(path, new_acc):
    with open(path) as json_file:
        input_score = json.load(json_file)

    init_pos = input_score['init_pos']
    fingers = input_score['fingers']
    phrase = input_score['phrase']

    for i in phrase:
        i['acceleration'] = new_acc
        i['deceleration'] = new_acc
    
    data = {'init_pos': init_pos, 'phrase': phrase, 'fingers': fingers, 'timestamp': datetime.now().strftime("%d/%m/%Y %H:%M:%S")}
    with open(path, 'w') as file:
        json.dump(data, file, indent=4, sort_keys=True)
    if 'recent_saves.txt' in os.listdir(base_path):
        with open(base_path + '/recent_saves.txt', 'r+') as file:
            content = file.read()
            file.seek(0, 0)
            file.write(path + '\n' + content)
    else:
        with open(base_path + '/recent_saves.txt', 'w') as file:
            file.write(path)
    
                   
def add_correction(path, correccion_radio, correccion_theta, correccion_offset, correction_flow, new_path):
    global MAX_ACC, base_path
    
    with open(path) as json_file:
        input_score = json.load(json_file)
    
    init_pos = input_score['init_pos']
    fingers = input_score['fingers']
    phrase = input_score['phrase']

    init_pos['r'] += correccion_radio
    init_pos['theta'] += correccion_theta
    init_pos['offset'] += correccion_offset

    for i in phrase:
        i['r'] += correccion_radio
        i['theta'] += correccion_theta
        i['offset'] += correccion_offset
        i['flow'] += correction_flow

    data = {'init_pos': init_pos, 'phrase': phrase, 'fingers': fingers, 'timestamp': datetime.now().strftime("%d/%m/%Y %H:%M:%S")}
    with open(new_path, 'w') as file:
        json.dump(data, file, indent=4, sort_keys=True)
    if 'recent_saves.txt' in os.listdir(base_path):
        with open(base_path + '/recent_saves.txt', 'r+') as file:
            content = file.read()
            file.seek(0, 0)
            file.write(new_path + '\n' + content)
    else:
        with open(base_path + '/recent_saves.txt', 'w') as file:
            file.write(new_path)



if __name__ == "__main__":
    path = "/home/fernando/Dropbox/UC/Magister/robot-flautista/exercises/bumblebee_super_fast.json"
    #print('En score_tools.py basepath:', base_path)

    change_all_acc(path, 1000)

    # for i in notes:
    #     for j in notes:
    #         print(f'{i}-{j} takes at least {min_time(i, j, 100)} sec')
    # change_speed(path, 2, path)