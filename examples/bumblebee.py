import json
import os
from datetime import datetime

notas = [
{
"note": "E5",
"time": 0.25
},
{
"note": "D#5",
"time": 0.25
},
{
"note": "D5",
"time": 0.25
},
{
"note": "C#5",
"time": 0.25
}, ## __
{
"note": "D5",
"time": 0.25
},
{
"note": "C#5",
"time": 0.25
},
{
"note": "C5",
"time": 0.25
},
{
"note": "B4",
"time": 0.25
}, ## __
{
"note": "C5",
"time": 0.25
},
{
"note": "B4",
"time": 0.25
},
{
"note": "A#4",
"time": 0.25
},
{
"note": "A4",
"time": 0.25
}, ## __
{
"note": "G#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
},
{
"note": "F4",
"time": 0.25
}, ## __
{
"note": "E4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
},
{
"note": "D4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
}, ## __
{
"note": "D4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
},
{
"note": "C4",
"time": 0.25
},
{
"note": "B3",
"time": 0.25
}, ## __
{
"note": "C4",
"time": 0.25
},
{
"note": "B3",
"time": 0.25
},
{
"note": "A#3",
"time": 0.25
},
{
"note": "A3",
"time": 0.25
}, ## __
{
"note": "G#3",
"time": 0.25
},
{
"note": "G3",
"time": 0.25
},
{
"note": "F#3",
"time": 0.25
},
{
"note": "F3",
"time": 0.25
}, ## __
{
"note": "E3",
"time": 0.5
}, ## Silencio M1
{
"note": "E4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
},
{
"note": "D4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
}, ## __
{
"note": "C4",
"time": 0.25
},
{
"note": "F4",
"time": 0.25

},
{
"note": "E4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
}, ## __
{
"note": "E4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
},
{
"note": "D4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
}, ## __
{
"note": "C4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
},
{
"note": "D4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
}, ## __
{
"note": "E4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
},
{
"note": "D4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
}, ## __
{
"note": "C4",
"time": 0.25
},
{
"note": "F4",
"time": 0.25
},
{
"note": "E4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
}, ## __
{
"note": "E4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
},
{
"note": "D4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
}, ## __
{
"note": "C4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
},
{
"note": "D4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
}, ## __ M2
{
"note": "E4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
},
{
"note": "D4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
}, ## __
{
"note": "D4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
},
{
"note": "C4",
"time": 0.25
},
{
"note": "B3",
"time": 0.25
}, ## __
{
"note": "C4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
},
{
"note": "D4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
}, ## __
{
"note": "E4",
"time": 0.25
},
{
"note": "F4",
"time": 0.25
},
{
"note": "E4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
}, ## __ M2
{
"note": "E4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
},
{
"note": "D4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
}, ## __
{
"note": "D4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
},
{
"note": "C4",
"time": 0.25
},
{
"note": "B3",
"time": 0.25
}, ## __
{
"note": "C4",
"time": 0.25
},
{
"note": "C#4",
"time": 0.25
},
{
"note": "D4",
"time": 0.25
},
{
"note": "D#4",
"time": 0.25
}, ## __
{
"note": "E4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
}, ## __ M3
{
"note": "A4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
}, ## __
{
"note": "F4",
"time": 0.25
},
{
"note": "A#4",
"time": 0.25
},
{
"note": "A4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
}, ## __
{
"note": "A4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
}, ## __
{
"note": "F4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
}, ## __ mov 3
{
"note": "A4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
}, ## __
{
"note": "F4",
"time": 0.25
},
{
"note": "A#4",
"time": 0.25
},
{
"note": "A4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
}, ## __
{
"note": "A4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
}, ## __
{
"note": "F4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
}, ## __ mov 4
{
"note": "A4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
}, ## __
{
"note": "G4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
},
{
"note": "F4",
"time": 0.25
},
{
"note": "E4",
"time": 0.25
}, ## __
{
"note": "F4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
}, ## __
{
"note": "A4",
"time": 0.25
},
{
"note": "A#4",
"time": 0.25
},
{
"note": "A4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
}, ## __ mov 4
{
"note": "A4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
}, ## __
{
"note": "G4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
},
{
"note": "F4",
"time": 0.25
},
{
"note": "E4",
"time": 0.25
}, ## __
{
"note": "F4",
"time": 0.25
},
{
"note": "F#4",
"time": 0.25
},
{
"note": "G4",
"time": 0.25
},
{
"note": "G#4",

"time": 0.25
}, ## __
{
"note": "A4",
"time": 0.25
},
{
"note": "A#4",
"time": 0.25
},
{
"note": "A4",
"time": 0.25
},
{
"note": "G#4",
"time": 0.25
}, ## __
{
"note": "A4",
"time": 0.5
}
]

init_pos = {
        "offset": 0.0,
        "r": 10.5,
        "theta": 45.0,
        "type": 0
    }

for n in notas:
    n['type'] = 2

phrase = []

filename = '/home/fernando/Dropbox/UC/Magister/robot-flautista/examples/bumblebee_vacio.json'
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