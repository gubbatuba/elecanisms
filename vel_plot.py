# vel_plot.py
#! /usr/bin/env python
import sys
import time
import struct
import csv
from usb_haptic import USBCommunications

def read_pos():
    vel = USB_COMMS.read_damper_vel()
    return vel

USB_COMMS = USBCommunications()

test_time = 10
sample_period = .01
csvname = 'data/haptic_damper0.csv'
time_start = time.clock()
elapsed_time = 0
data = []
print "Reading..."
while elapsed_time < test_time:
    time_read_start = time.clock()
    vel, dc = read_vel()
    rel_time = time_read_start - time_start
    elapsed_time = rel_time
    this_data = {
        "time": rel_time,
        "velocity": vel,
        "duty_cycle": dc
    }
    data.append(this_data)
    while time.clock() < time_read_start + sample_period:
        pass

print "Writing data..."
with open(csvname, 'wb') as datasheet:
    fieldnames = ['time', 'velocity', 'duty_cycle']
    writer = csv.DictWriter(datasheet, fieldnames=fieldnames)

    writer.writeheader()
    for reading in data:
        writer.writerow(reading)
