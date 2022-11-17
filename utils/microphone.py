import numpy as np
from regex import E
import sounddevice as sd
from scipy import signal
#from YIN.yin import compute_yin
import time
import threading
import random
from functools import partial
from librosa import yin
from scipy.io.wavfile import write

class Microphone(threading.Thread):
    def __init__(self, running):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.running = running
        self.sr = 22050
        self.max_num_points = int(self.sr*1)
        self.last_mic_data = np.array([])
        self.last = []
        self.pitch = 0
        self.flt = signal.remez(121, [0, 50, 240, int(self.sr/2)], [0, 1], fs=self.sr)
        self.A = [1] +  [0 for i in range(77-1)]
        fo = 12800
        l  = 0.995
        self.B2  = [1, -2*np.cos(2*np.pi*fo/self.sr), 1]
        self.A2  = [1, -2*l*np.cos(2*np.pi*fo/self.sr), l**2]
        self.saving = False
        self.data = np.array([])

    def micCallback(self, indata, frames, time, status):
        if status:
            print('Status:', status)
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
        with sd.InputStream(samplerate=self.sr, channels=1, callback=self.micCallback, latency='high'): #, device=6
            while self.running.is_set():
                sd.sleep(50)
                #pitches, harmonic_rates, argmins, times = compute_yin(self.last_mic_data, self.sr, f0_max=2000)#, w_len=int(len(self.last_mic_data)-1), harmo_thresh=0.1,f0_max=self.sr/2, w_step=int(len(self.last_mic_data)-1)) 
                senal_filtrada1 = signal.lfilter(self.flt, self.A, self.last_mic_data)
                senal_filtrada2 = signal.lfilter(self.B2, self.A2, senal_filtrada1)

                pitches = yin(senal_filtrada2, sr=self.sr, fmin=100, fmax=12800) #, trough_threshold=0.0001)]
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
    event = threading.Event()
    event.set()

    mic = Microphone(event)
    mic.start()

    while True:
        n = input()
        if n=='e':
            event.clear()
            break