#! /usr/bin/env python
import sys
import time
import struct

from usb_haptic import USBCommunications

USB_COMMS = USBCommunications()

while 1:
    t0 = time.clock()
    sys.stdout.write('\x1b[2J\x1b[1;1f')
    encoder_reading = USB_COMMS.enc_readReg(0x3FFF)
    print encoder_reading
    encoder_reading[1] = encoder_reading[1] & 0x3F
    print encoder_reading
    print struct.unpack("<H", encoder_reading.tostring())
    while time.clock() < t0 + 0.05:
        pass
