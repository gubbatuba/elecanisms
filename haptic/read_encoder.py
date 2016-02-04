
import encodertest
import sys, time
import struct
foo = encodertest.encodertest()

while 1:
    sys.stdout.write('\x1b[2J\x1b[1;1f')

    enc = foo.enc_readReg(0x3FFF)
    print "START"
    print
    print enc
    # print enc.tolist()
    t0 = time.clock()
    # binlist = [bin(x)[2:] for x in enc.tolist()]
    # print "BINLIST: ", binlist
    # for i, b in enumerate(binlist):
    #     print i,b
    #     if len(binlist[i]) < 8:
    #         diff = 8 - len(binlist[i])
    #         binlist[i] = '0'*diff + binlist[i]

    # print "BINLIST2: ", binlist
    # full_data = binlist[0] + binlist[1]
    # angle_data = int(full_data[2:], 2)

    # print binlist
    # print angle_data
    # bit mask
    # unsigned int little endian
    enc[1]
    enc[1] = enc[1] & 0x3F
    print struct.unpack("<H", enc.tostring())
    print "END"
    while time.clock()<t0+0.05:
        pass

