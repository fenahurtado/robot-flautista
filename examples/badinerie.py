
import json
import os
from datetime import datetime

pentagrama = [
    ("B4", 0.5), ("D5", 0.25), ("B4", 0.25), ("F#4", 0.5), ("B5", 0.25), ("F#4", 0.25), ("D4", 0.5), ("F#4", 0.25), ("D4", 0.25), ("B3", 1),
#
    ("F#3", 0.25), ("B3", 0.25), ("D4", 0.25), ("B3", 0.25), ("C#4", 0.25), ("B3", 0.25), ("C#4", 0.25), ("B3", 0.25), ("A#3", 0.25), ("C#4", 0.25), ("E4", 0.25), ("C#4", 0.25), ("D4", 0.5), ("B3", 0.5),
#
    ("B4", 0.5), ("D5", 0.25), ("B4", 0.25), ("F#4", 0.5), ("B5", 0.25), ("F#4", 0.25), ("D4", 0.5), ("F#4", 0.25), ("D4", 0.25), ("B3", 1)
]

notas = []
for i in pentagrama:
    notas.append({"note": i[0],
                  "time": i[1],
                  "type": 2})

phrase = []

init_pos = {
        "offset": 0.0,
        "r": 10.5,
        "theta": 45.0,
        "type": 0
    }

filename = '/home/fernando/Dropbox/UC/Magister/robot-flautista/examples/badinerie.json'
base_path = '/home/fernando/Dropbox/UC/Magister/robot-flautista/view_control'


data = {'init_pos': init_pos, 'phrase': phrase, 'fingers': notas, 'timestamp': datetime.now().strftime("%d/%m/%Y %H:%M:%S")}
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