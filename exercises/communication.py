import sys
import struct
import time
from multiprocessing import Process, Event, Pipe, Value, Manager
sys.path.insert(0, 'C:/Users/ferna/Dropbox/UC/Magister/robot-flautista')
import lib.ethernet_ip.ethernetip as ethernetip
#from exercises.drivers_connect import Command, Setting, INPUT_FUNCTION_BITS, VirtualFlow, VirtualAxis
from numpy import sign

class CommunicationCenter(Process):
    def __init__(self, host, event, pipe, data, connect=True, verbose=False):
        Process.__init__(self)
        self.event = event
        self.pipe = pipe
        self.host = host
        self.EIP = ethernetip.EtherNetIP(self.host)
        self.connections = {}
        self.data = data
        self.verbose = verbose
        self.connect = connect
    
    def run(self):
        self.EIP.startIO()
        while self.event.is_set():
            for host, conn in self.connections.items():
                try:
                    self.data[host + '_in'] = conn.inAssem
                    conn.outAssem = self.data[host + '_out']
                except:
                    print("Hubo un error en la lectura del input en el centro de comunicaciones")
                    print(self.data)
            if self.pipe.poll():
                message = self.pipe.recv()
                if self.verbose:
                    print("Message received", message)
                if message[0] == "explicit_conn":
                    C1 = self.EIP.explicit_conn(message[1])
                    C1.outAssem = [0 for i in range(message[2])]
                    C1.inAssem = [0 for i in range(message[3])]

                    pkt = C1.listID()
                    if pkt is not None:
                        print("Product name: ", pkt.product_name.decode())

                    inputsize = message[4]
                    outputsize = message[5]

                    self.data[message[1] + '_in'] = [0 for i in range(inputsize*8)]
                    self.data[message[1] + '_out'] = [0 for i in range(outputsize*8)]

                    self.EIP.registerAssembly(message[6], inputsize, message[7], C1)
                    self.EIP.registerAssembly(message[8], outputsize, message[9], C1)

                    self.connections[message[1]] = C1
                    print("Assembly registered...")

                elif message[0] == "registerSession":
                    self.connections[message[1]].registerSession()

                elif message[0] == "sendFwdOpenReq":
                    self.connections[message[1]].sendFwdOpenReq(message[2], message[3], message[4], torpi=message[5], otrpi=message[6], priority=message[7])
                    self.connections[message[1]].produce()

                elif message[0] == "stopProduce":
                    self.connections[message[1]].stopProduce()
                    self.connections[message[1]].sendFwdCloseReq(message[2], message[3], message[4])
                    del self.connections[message[1]]
                
                elif message[0] == "setAttrSingle":
                    self.connections[message[1]].setAttrSingle(message[2], message[3], message[4], message[5])
                
                elif message[0] == "getAttrSingle":
                    att = self.connections[message[1]].getAttrSingle(message[2], message[3], message[4])
                    self.pipe.send([message[1], att])
            time.sleep(0.008)
        self.EIP.stopIO()


if __name__ == "__main__":
    host = "192.168.2.10"
    devices = ["192.168.2.102", "192.168.2.104", "192.168.2.103"]

    event1 = Event()
    event1.set()
    event2 = Event()
    event2.set()

    main_pipe1, main_pipe2 = Pipe()
    comm_pipe1, comm_pipe2 = Pipe()

    mgr = Manager()
    comm_data = mgr.dict()

    cc = CommunicationCenter(host, event1, comm_pipe1, comm_data, verbose=True)
    cc.start()

    t0 = time.time()
    
    # x_virtual_axis = VirtualAxis(event2, 0.01, t0, x_virtual_axis_end_conn, verbose=False)
    # x_virtual_axis.start()
    x_virtual_axis = None
    
    x_driver_conn, x_driver_end_conn = Pipe()
    x_virtual_axis_conn, x_virtual_axis_end_conn = Pipe()

    x_driver = AMCIDriver(devices[0], event2, x_virtual_axis, x_driver_end_conn, comm_pipe2, comm_data, x_virtual_axis_end_conn, t0, connected=True, starting_speed=1, verbose=False, input_2_function_bits=INPUT_FUNCTION_BITS['CW Limit'], virtual_axis_follow_acceleration=400, virtual_axis_follow_deceleration=400, home=True, use_encoder_bit=1, motor_current=40, virtual_axis_proportional_coef=1, encoder_pulses_turn=4000, motors_step_turn=4000, hybrid_control_gain=0, enable_stall_detection_bit=0, current_loop_gain=5, Kp=0, Ki=5, Kd=0.01)
    x_driver.start()
    # ps = PressureSensor(devices[0], event2, comm_pipe2, comm_data, connected=True, verbose=False)
    # ps.start()

    # t0 = time.time()
    # mfc = FlowControllerDriver(devices[1], event2, t0, main_pipe2, comm_pipe2, comm_data, verbose=False)
    # mfc.start()
    
    while True:
        m = input()
        if m == "q":
            event2.clear()
            time.sleep(0.1)
            event1.clear()
            break
        elif m == "m":
            tn = time.time() + 1
            ref = [(tn,100,0)]
            x_virtual_axis_conn.send(["merge_ref", ref])
        else:
            print(2)
    
    print("Closing...")