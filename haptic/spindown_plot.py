#! /usr/bin/env python
import sys
import time
import struct
import csv
from usb_haptic import USBCommunications

def ticks_to_degrees(ticks):
    return ticks/45.511111

def read_encoder():
    encoder_reading = USB_COMMS.enc_readReg(0x3FFF)
    encoder_reading[1] = encoder_reading[1] & 0x3F
    return struct.unpack("<H", encoder_reading.tostring())[0]

USB_COMMS = USBCommunications()
#Use button to stop motor

# sample = {
#     "time": None,
#     "ticks": None,
#     "degrees": None
# }
test_time = 10
sample_period = .001
csvname = 'spindowndata_001_95_fwd.csv'
time_start = time.clock()
elapsed_time = 0
data = []
print "Reading..."
while elapsed_time < test_time:
    time_read_start = time.clock()
    ticks = read_encoder()
    degrees = ticks_to_degrees(ticks)
    rel_time = time_read_start - time_start
    elapsed_time = rel_time
    this_data = {
        "time": rel_time,
        "ticks": ticks,
        "degrees": degrees
    }
    data.append(this_data)
    while time.clock() < time_read_start + sample_period:
        pass

print "Writing data..."
with open(csvname, 'wb') as datasheet:
    fieldnames = ['time', 'ticks', 'degrees']
    writer = csv.DictWriter(datasheet, fieldnames=fieldnames)

    writer.writeheader()
    for reading in data:
        writer.writerow(reading)
