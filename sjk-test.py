from bluepy.btle import UUID, Peripheral, DefaultDelegate, AssignedNumbers
import struct
import math

import time
import sys
import argparse


def _TI_UUID(val):
    return UUID("%08X-0451-4000-b000-000000000000" % (0xF0000000+val))

SJK_MAC = 'a0:e6:f8:ae:14:00'
#SJK_MAC = '24:71:89:E9:72:07'

class SensorBase:
 
    def __init__(self, periph):
        self.periph = periph
        self.service = None
        self.data0 = None
        self.data1 = None

    def enable(self):
        if self.service is None:
            print(self.svcUUID)
            self.service = self.periph.getServiceByUUID(self.svcUUID)
            print("end service find")
            print(self.service)
        if self.data0 is None:
            self.data0 = self.service.getCharacteristics(self.data0UUID)[0]
        if self.data1 is None:
            self.data1 = self.service.getCharacteristics(self.data1UUID)[0]


    def read0(self):
        return self.data0.read()

    def read1(self):
        return self.data1.read()



class SjkSensor(Peripheral):

    svcUUID = ("%08X-0451-4000-b000-000000000000" % (0xF0000000+0x1220))
    data0UUID = ("%08X-0451-4000-b000-000000000000" % (0xF0000000+0x1221))
    data1UUID = ("%08X-0451-4000-b000-000000000000" % (0xF0000000+0x1222))

    def __init__(self, addr):
        Peripheral.__init__(self,addr)
        self.periph = Peripheral;
        self.service = None
        self.data0 = None
        self.data1 = None
        self.noti_cntl = None

    def enable(self):
        if self.service is None:
            print(self.svcUUID)
            self.service = self.getServiceByUUID(self.svcUUID)
            print("Service find")
            print(self.service)
        if self.data0 is None:
            self.data0 = self.service.getCharacteristics(self.data0UUID)[0]
        if self.data1 is None:
            self.data1 = self.service.getCharacteristics(self.data1UUID)[0]

    def notienable(self):
        if self.noti_cntl is None:
            self.noti_cntl = self.service.getDescriptors(forUUID=0x2902)[0]
            self.noti_cntl.write(struct.pack('<bb', 0x01, 0x00), True)

    def notidisable(self):
        if self.noti_cntl is not None:
            self.noti_cntl.write(struct.pack('<bb', 0x00, 0x00), True)

    def read1(self):
        return self.data1.read()

class SjkDelegate(DefaultDelegate):
 
    def __init__(self):
        DefaultDelegate.__init__(self)
        self.lastVal = 0

    def handleNotification(self, hnd, data):

        val = struct.unpack("B", data)[0]
        print("noti value = %d" % val)


import signal
import sys


def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)


def main():

    signal.signal(signal.SIGINT, signal_handler)

    print('Connecting to ' + SJK_MAC)
    sjk = SjkSensor(SJK_MAC)
    svcs = sjk.discoverServices()

    # Enabling selected sensors
    sjk.enable()

    sjk.setDelegate(SjkDelegate())
    sjk.notienable()
    while True:
        print("wait for the noti")
        sjk.waitForNotifications(100)

        rcvData = sjk.read1()
        kk = struct.unpack('@300h', rcvData)
        print(repr(kk))
        print('size of rcvdata')
        print("%s" % len(rcvData))

    print('exit loop with timeout')

    '''    
    rcvData = sjk.read1()
    kk = struct.unpack('!300H', rcvData)
    print(repr(kk))
    print('size of rcvdata')
    print("%s" % len(rcvData))
    '''

    sjk.disconnect()
    del sjk

Exception





if __name__ == "__main__":
    main()
