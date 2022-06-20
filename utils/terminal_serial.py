## sudo code --verbose --user-data-dir --no-sandbox 
## sudo chmod 666 /dev/ttyUSB0  
## see - https://stackoverflow.com/questions/5404068/how-to-read-keyboard-input/53344690#53344690

import serial
import threading
import queue
import time
from utils.cinematica import *

class SerialPort(threading.Thread):
    def __init__(self, verbose=True):
        threading.Thread.__init__(self)
        self.verbose = verbose
        self.EXIT_COMMAND = "EXIT"
        self.inputQueue = queue.Queue()
        self.started = False

        self.ser = serial.Serial()
        self.ser.baudrate = 115200
        self.ser.port = '/dev/ttyUSB0'
        #ser.timeout = 1
        self.ser.open()

    def run(self):
        while (True):
            if (self.inputQueue.qsize() > 0):
                input_str = self.inputQueue.get().upper()
                if self.verbose:
                    print("input_str = {}".format(input_str))

                if (input_str == self.EXIT_COMMAND):
                    print("Exiting serial terminal.")
                    break
                else:
                    encoded = str.encode(input_str)
                    encoded += b'\n'
                    self.ser.write(encoded)
                # Insert your code here to do whatever you want with the input_str.

            if (self.ser.inWaiting() > 0):
                # read the bytes and convert from binary array to ASCII
                data_str = self.ser.read(self.ser.inWaiting()).decode('ascii') 
                # print the incoming string without putting a new-line
                # ('\n') automatically after every print()
                if "start" in data_str:
                    self.started = True
                if self.verbose:
                    print('>>', data_str, end='') 

            time.sleep(0.01)

        self.ser.close()
        print("End.")
    
    def write(self, inst):
        self.inputQueue.put(inst)

class FakeSerialPort:
    def __init__(self, verbose=True):
        self.started = True
        self.verbose = verbose

    def write(self, value):
        if self.verbose:
            print(value)
        
if (__name__ == '__main__'): 
    sPort = SerialPort()
    sPort.start()
    input()
    sPort.write('g28')
    while True:
        input()
        sPort.write(f'g01 x{x_mm2units(8.49)} y{z_mm2units(8.48)} z{alpha_angle2units(20)} f600')
        input()
        sPort.write('g01 x0 y0 z0 f600')

# 600 units/min
# 600 * 80 mcsteps / min
# 600 * (80 / 1600) vueltas / min
# 600 * (80 / 1600) * 8 mm / min
# 240 mm / min = 4 mm / s

# 600 * (X_PASOS_x_UNIDAD / X_MICROSTEPS) * X_MM_x_REVOLUCION mm / min
