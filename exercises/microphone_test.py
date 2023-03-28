import sys

import pandas as pd
import sounddevice as sd
from scipy import signal
from librosa import yin
from scipy.io.wavfile import write

sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
import threading
import lib.ethernet_ip.ethernetip as ethernetip
from utils.motor_route import *
from utils.driver_fingers import FingersDriver
import struct
import time
import numpy as np

class Microphone(threading.Thread):
    def __init__(self, running, connected=False, verbose=False):
        threading.Thread.__init__(self)
        self.running = running
        self.connected = connected
        self.verbose = verbose
        self.pitch = 0
        self.sr = 44100
        self.max_num_points = int(self.sr*0.1)
        self.last_mic_data = np.array([])
        self.last = []
        self.flt = signal.remez(121, [0, 50, 240, int(self.sr/2)], [0, 1], fs=self.sr)
        self.A = [1] +  [0 for i in range(77-1)]
        fo = 12800
        l  = 0.995
        self.B2  = [1, -2*np.cos(2*np.pi*fo/self.sr), 1]
        self.A2  = [1, -2*l*np.cos(2*np.pi*fo/self.sr), l**2]
        self.saving = False
        self.data = np.array([])
        self.print_i = 0

    def micCallback(self, indata, frames, time, status):
        self.print_i = (self.print_i + 1) % 20
        print(self.print_i)
        if status:
            print('Status:', status)
        #senal_filtrada1 = signal.lfilter(self.flt, self.A, indata)
        #senal_filtrada2 = signal.lfilter(self.B2, self.A2, senal_filtrada1)
        self.last_mic_data = np.hstack((self.last_mic_data, np.transpose(indata)[0]))
        self.last_mic_data = self.last_mic_data[-self.max_num_points:]
        if self.saving:
            self.data = np.hstack((self.data, np.transpose(indata)[0]))
            #print(self.data.size)

    def start_saving(self):
        self.data = np.array([])
        self.saving = True
    
    def pause_saving(self):
        self.saving = False

    def resume_saving(self):
        self.saving = True
    
    def finish_saving(self, file_name):
        self.saving = False
        print(self.data.size)
        write(file_name, self.sr, self.data)
        #self.data.to_csv(file_name)

    def run(self):
        if self.connected:
            with sd.InputStream(samplerate=self.sr, channels=1, callback=self.micCallback, device=6, latency='high'): #, latency='high',  blocksize=10
                while self.running.is_set():
                    sd.sleep(50)
                    #pitches, harmonic_rates, argmins, times = compute_yin(self.last_mic_data, self.sr, f0_max=2000)#, w_len=int(len(self.last_mic_data)-1), harmo_thresh=0.1,f0_max=self.sr/2, w_step=int(len(self.last_mic_data)-1)) 
                    #senal_filtrada1 = signal.lfilter(self.flt, self.A, self.last_mic_data)
                    #senal_filtrada2 = signal.lfilter(self.B2, self.A2, senal_filtrada1)

                    pitches = yin(self.last_mic_data, sr=self.sr, fmin=100, fmax=12800) #, trough_threshold=0.0001)]
                    #print(pitches[-1])
                    #print(1/(self.last_mic_data.shape[0]*(1/self.sr)), pitches[-1])
                    #compute_yin() NUT = 1
                    # N = 44100*1
                    # T = 1/44100
                    # U = 1
                    #print(pitches, len(self.last_mic_data))
                    self.pitch = pitches[-1]
                    #print(self.pitch)
                    #self.last_mic_data = np.array([])
                    #self.callback.doc.add_next_tick_callback(partial(self.callback.update2, self.last))

            
            print("Mic thread killed")

if __name__ == "__main__":
    print(sd.query_devices())
    event = threading.Event()
    event.set()
    mic = Microphone(event, connected=True, verbose=True)
    mic.start()

    while True:
        t = input()
        if t == "q":
            event.clear()
            break
        elif t == "r":
            mic.start_saving()
        elif t == "s":
            mic.finish_saving("/home/fernando/Dropbox/UC/Magister/robot-flautista/exercises/data/escala2.wav")