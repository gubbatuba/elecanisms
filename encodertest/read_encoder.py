
import encodertest
import sys, time

foo = encodertest.encodertest()

while 1:
    sys.stdout.write('\x1b[2J\x1b[1;1f')

    enc = foo.enc_readReg(0x3FFF)
    print "START"
    print
    print enc
    print
    print "END"
    t0 = time.clock()
    while time.clock()<t0+0.05:
        pass

