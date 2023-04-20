import sys
import struct
sys.path.insert(0, 'C:/Users/ferna/Dropbox/UC/Magister/robot-flautista')
import lib.ethernet_ip.ethernetip as ethernetip



if __name__ == "__main__":
    host = "192.168.2.10"
    hostname = "192.168.2.100"
    #connections = ["192.168.2.102", "192.168.2.104", "192.168.2.103", "192.168.2.101", "192.168.2.100"]

    EIP = ethernetip.EtherNetIP(host)
    print("EIP created...")

    EIP.startIO()
    print("EIP started...")

    C1 = EIP.explicit_conn(hostname)
    #C1.inAssem = [0 for i in range(4*8)]
    print("Explicit connection created...")

    pkt = C1.listID()
    if pkt is not None:
        print("Product name: ", pkt.product_name.decode())

    inputsize = 10
    outputsize = 4

    # self.C1.outAssem = [False for i in range(8*4)]

    # configure i/o
    # print("Configure with {0} bytes input and {1} bytes output".format(inputsize, outputsize))
    EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_INPUT, inputsize, 101, C1)
    EIP.registerAssembly(ethernetip.EtherNetIP.ENIP_IO_TYPE_OUTPUT, outputsize, 100, C1)

    C1.registerSession()
            
    C1.sendFwdOpenReq(101, 100, 0x6e, torpi=50, otrpi=50)#, priority=ethernetip.ForwardOpenReq.FORWARD_OPEN_CONN_PRIO_HIGH)
    C1.produce()
        
    print("Assembly registered...")
    while True:
        if input() == "q":
            break
    
    print("Closing...")
    C1.stopProduce()
    C1.sendFwdCloseReq(101, 100, 0x6e)

    EIP.stopIO()
    print("Closed.")