import sys
sys.path.insert(0, '/home/fernando/Dropbox/UC/Magister/robot-flautista')
from eeip.eeip import *
import time
import struct

eeipclient = EEIPClient()
#Ip-Address of the Ethernet-IP Device (In this case Allen-Bradley 1734-AENT Point I/O)
#A Session has to be registered before any communication can be established
eeipclient.register_session('192.168.2.103')

#Parameters from Originator -> Target
# 0x20, 0x04,0x24, 0x6E, 0x2C, 0x96, 0x2C, 0x64
# 0x20, 0x04,0x24, 0x01, 0x2C, 0x64, 0x2C, 0x65
eeipclient.configuration_assembly_instance_id = 0x6e

eeipclient.o_t_instance_id = 0x96
eeipclient.o_t_length = 20
eeipclient.o_t_requested_packet_rate = 100000  #Packet rate 100ms (default 500ms)
eeipclient.o_t_realtime_format = RealTimeFormat.HEADER32BIT
eeipclient.o_t_owner_redundant = False
eeipclient.o_t_variable_length = False
eeipclient.o_t_connection_type = ConnectionType.POINT_TO_POINT

#Parameters from Target -> Originator
eeipclient.t_o_instance_id = 0x64
eeipclient.t_o_length = 20
eeipclient.t_o_requested_packet_rate = 100000  #Packet rate 100ms (default 500ms)
eeipclient.t_o_realtime_format = RealTimeFormat.MODELESS
eeipclient.t_o_owner_redundant = False
eeipclient.t_o_variable_length = False
eeipclient.t_o_connection_type = ConnectionType.POINT_TO_POINT
eeipclient.o_t_iodata = [0 for i in range(200)]

def process_data(data):
    if len(data) == 20:
        b = struct.pack('20B', *data)
        [w0, w1, w2, w3, w4, w5, w6, w7, w8, w9] = struct.unpack('<HHHHHHHHHH', b) # gas, status, absolut pressure, flow temperature, volumetric flow, mass flow, mass flow set point
        print(w0)

#Forward open initiates the Implicit Messaging
eeipclient.forward_open()
while 1:
    process_data(eeipclient.t_o_iodata)
    #print(eeipclient.o_t_iodata)
    time.sleep(0.5)

#Close the Session (First stop implicit Messaging then unregister the session)
eeipclient.forward_close()
eeipclient.unregister_session()