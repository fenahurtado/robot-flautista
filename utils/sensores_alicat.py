import time
import threading
import random
from functools import partial
from time import sleep
from cpppo.server.enip import poll
from cpppo.server.enip.get_attribute import proxy_simple as device
from PyQt5 import QtCore

class FlowController(QtCore.QThread):
    SCALE = 2
    flow_change_signal = QtCore.pyqtSignal(object)
    
    def __init__(self, host, running, connected=True):
        QtCore.QThread.__init__(self)
        self.connected = connected

        self.params                  = [('@4/101/3',("INT", "DINT", "REAL", "REAL", "REAL", "REAL", "REAL"))]

        self.hostname                = host
        self.values                  = {"gass": 0, "status": 0, "pressure": 0, "temperature": 0, "vol_flow": 0, "mass_flow": 0, "set_point": 0}
        self.poller                  = threading.Thread(
            target=poll.poll, args=(device,), kwargs={
                'address':      (self.hostname, 44818),
                'cycle':        0.05,
                'timeout':      0.5,
                'process':      self.process_incoming_data,
                'params':       self.params,
            })
        self.poller.daemon           = True
        if self.connected:
            self.poller.start()

        # self.changeEvent = threading.Event()
        # self.host = '192.168.2.101'
        if self.connected:
            self.via = device(self.hostname)
        else:
            self.via = None

        # self.val = [0 for i in range(7)] # Set default sensor data to be zero
        self.running = running # Store the current state of the Flag
        
        self.ref_comunication = [True, 0]
        self.controlled_var = [True, 0]
        self.P_communication = [True, 345]
        self.I_communication = [True, 0]
        self.D_communication = [True, 2816]
        self.CL_comunication = [True, 0]

    def process_incoming_data(self, par, val):
        self.values.update( { "gass": val[0], "status": val[1], "pressure": val[2], "temperature": val[3], "vol_flow": val[4], "mass_flow": val[5], "set_point": val[6] } )
        self.flow_change_signal.emit(val[5])

    def assembly102(self, cmdID, Argument):
        with self.via:
            data, = self.via.read( [(f'@4/102/3=(INT){cmdID},{int(Argument)}', ("INT", "INT"))])
    
    def assembly103(self):
        with self.via:
            data, = self.via.read( [('@4/103/3',("INT", "INT"))] )
        return data

    def change_ref(self, value):
        if value != self.ref_comunication[1]:
            self.ref_comunication[1] = value
            self.ref_comunication[0] = True

    def change_controlled_var(self, value):
        if value == 'V':
            self.controlled_var[1] = 0
        elif value == 'M':
            self.controlled_var[1] = 1
        elif value == 'P':
            self.controlled_var[1] = 3
        self.controlled_var[0] = True
        
    def change_control_loop(self, value):
        self.CL_comunication[1] = value
        self.CL_comunication[0] = True

    def change_kp(self, value):
        self.P_communication[1] = value
        self.P_communication[0] = True

    def change_ki(self, value):
        self.I_communication[1] = value
        self.I_communication[0] = True

    def change_kd(self, value):
        self.D_communication[1] = value
        self.D_communication[0] = True

    def run(self):
        while self.running.is_set(): # Continue grabbing data from sensor while Flag is set
            if not self.connected:
                time.sleep(0.01)  # Time to sleep in seconds, emulating some sensor process taking time
                gas_2 = 0
                state_2 = 0
                press_cf = random.random()*FlowController.SCALE+FlowController.SCALE*1
                temp = random.random()*FlowController.SCALE+FlowController.SCALE*2
                volume = random.random()*FlowController.SCALE+FlowController.SCALE*3
                mass = random.random()*FlowController.SCALE+FlowController.SCALE*4
                ref = random.random()*FlowController.SCALE+FlowController.SCALE*5
                self.val = [gas_2, state_2, press_cf, temp, volume, mass, ref] # Generate random integers to emulate data from sensor
                # self.callbackFunc.doc.add_next_tick_callback(partial(self.callbackFunc.update, self.val)) # Call Bokeh webVisual to inform that new data is available
            else:
                if self.ref_comunication[0]:
                    #print('Sending: {}'.format(self.ref_comunication[1]/1))
                    with self.via:
                        data, = self.via.read( [('@4/100/3={}'.format(self.ref_comunication[1]/1),"REAL")] )
                    self.ref_comunication[0] = False
                if self.controlled_var[0]:
                    self.assembly102(11,self.controlled_var[1])
                    self.controlled_var[0] = False
                if self.P_communication[0]:
                    self.assembly102(8,self.P_communication[1])
                    self.P_communication[0] = False
                if self.I_communication[0]:
                    self.assembly102(10,self.I_communication[1])
                    self.I_communication[0] = False
                if self.D_communication[0]:
                    self.assembly102(9,self.D_communication[1])
                    self.D_communication[0] = False
                if self.CL_comunication[0]:
                    #print(CL_comunication[0])
                    self.assembly102(13,self.CL_comunication[1])
                    self.CL_comunication[0] = False
                #self.val = []
                # with self.via:
                #     data2, = self.via.read( [('@4/101/3',("INT", "DINT", "REAL", "REAL", "REAL", "REAL", "REAL"))] )
                # self.val[3:] = data2
                #print(self.val)
                sleep(0.05)
        if self.connected:
            with self.via:
                data, = self.via.read( [('@4/100/3=0.0',"REAL")] )
            print("Flow Controller thread killed") # Print to indicate that the thread has ended

class PreasureSensor(threading.Thread):
    SCALE = 2
    
    def __init__(self, host, running, connected=True):
        threading.Thread.__init__(self) # Initialize the threading superclass
        self.connected = connected

        self.params                  = [('@4/101/3',("INT", "DINT", "REAL"))]

        self.hostname                = host
        self.values                  = {"pressure": 0}
        self.poller                  = threading.Thread(
            target=poll.poll, args=(device,), kwargs={
                'address':      (self.hostname, 44818),
                'cycle':        0.01,
                'timeout':      0.5,
                'process':      lambda par,val: self.values.update( { "pressure": val[2] } ),
                'params':       self.params,
            })
        self.poller.daemon           = True
        
        #self.val = [0 for i in range(3)] # Set default sensor data to be zero
        self.running = running # Store the current state of the Flag
        
    def run(self):
        if self.connected:
            self.poller.start()
        while self.running.is_set(): # Continue grabbing data from sensor while Flag is set
            if not self.connected:
                sleep(0.01)  # Time to sleep in seconds, emulating some sensor process taking time
                gas_1 = 0
                state_1 = 0
                press_sf = random.random()*PreasureSensor.SCALE
        
                self.values['pressure'] = press_sf # Generate random integers to emulate data from sensor
                # self.callbackFunc.doc.add_next_tick_callback(partial(self.callbackFunc.update, self.val)) # Call Bokeh webVisual to inform that new data is available
            else:
                sleep(1)
                #print(self.values['pressure'])

        print("Pressure Sensor thread killed") # Print to indicate that the thread has ended