import ethernetip
import random
import socket
import struct
import time

import numpy as np


def main():
    hostname = "192.168.2.101"
    broadcast = "192.168.2.101"
    inputsize = 1
    outputsize = 1
    EIP = ethernetip.EtherNetIP(hostname)
    C1 = EIP.explicit_conn(hostname)

    listOfNodes = C1.scanNetwork(broadcast, 1)
    print("Found ", len(listOfNodes), " nodes")
    for node in listOfNodes:
        name = node.product_name.decode()
        sockinfo = ethernetip.SocketAddressInfo(node.socket_addr)
        ip = socket.inet_ntoa(struct.pack("!I", sockinfo.sin_addr))
        print(ip, " - ", name)

    pkt = C1.listID()
    if pkt is not None:
        print("Product name: ", pkt.product_name.decode())

    pkt = C1.listServices()
    print("ListServices:", str(pkt))

    # read input size from global system object (obj 0x84, attr 4)
    # r = C1.getAttrSingle(0x37, 1, 4)
    # if 0 == r[0]:
    #     print("Read CPX input size from terminal success (data: " + str(r[1]) + ")")
    inputsize = 26 #struct.unpack("B", r[1])[0]

    # read output size from global system object (obj 0x84, attr 5)
    # r = C1.getAttrSingle(0x37, 1, 5)
    # if 0 == r[0]:
    #     print("Read CPX output size from terminal sucess (data: " + str(r[1]) + ")")
    outputsize = 4 #struct.unpack("B", r[1])[0]

    # configure i/o
    print("Configure with {0} bytes input and {1} bytes output".format(inputsize, outputsize))
    EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, inputsize, 101, C1)
    EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, outputsize, 100, C1)
    EIP.startIO()

    C1.registerSession()

    #C1.setAttrSingle(ethernetip.CIP_OBJ_TCPIP, 1, 6, "fbxxx")

    for i in range(1, 8):
        r = C1.getAttrSingle(ethernetip.CIP_OBJ_IDENTITY, 1, i)
        if 0 == r[0]:
            print("read ok attr (" + str(i) + ") data: " + str(r[1]))
        else:
            print("Err: " + str(r[0]))

    C1.sendFwdOpenReq(101, 100, 0x6e, torpi=5, otrpi=5)
    C1.produce()
    
    def read_input():
        words = []
        for w in range(26):
            words.append(int("".join(["1" if C1.inAssem[i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
        words2 = []
        for w in range(4):
            words2.append(int("".join(["1" if C1.outAssem[i] else "0" for i in range(w*8+7, w*8-1, -1)]), 2))
        b = bytearray(26)
        struct.pack_into('26B', b, 0, *words)
        [g, s, ap, ft, vf, mf, mfsp] = struct.unpack('<HIfffff', b)
        b2 = bytearray(4)
        struct.pack_into('4B', b2, 0, *words2)
        [ref] = struct.unpack('<f', b2)
        
        print(g, s, ap, ft, vf, mf, mfsp, ref)

    def set_output(value):
        b = bytearray(4)
        struct.pack_into('f', b, 0, value)
        
        words = []
        for byte in b:
            words.append(''.join(format(byte, '08b'))[::-1])
        #

        for i in range(4):
            for j in range(8):
                C1.outAssem[i*8+j] = int(words[i][j]) == 1

    t0 = time.time()
    while True:
        try:
            time.sleep(0.05)
            t = time.time()-t0
            val = 5+5*np.sin(2*np.pi*3*t)
            set_output(val)
            read_input()
            
            #print(C1.outAssem)
            # C1.outAssem[random.randint(0, len(C1.outAssem) - 1)] = True
            # C1.outAssem[random.randint(0, len(C1.outAssem) - 1)] = False
        except KeyboardInterrupt:
            break
    C1.stopProduce()
    C1.sendFwdCloseReq(101, 100, 0x6e)
    EIP.stopIO()


if __name__ == "__main__":
    main()
