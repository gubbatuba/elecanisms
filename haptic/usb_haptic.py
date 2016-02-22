
import usb.core
import time
import struct


class USBCommunications(object):

    def __init__(self):
        self.TOGGLE_LED1 = 1
        self.TOGGLE_LED2 = 2
        self.TOGGLE_LED3 = 3
        self.READ_SW1 = 4
        self.READ_SW2 = 5
        self.READ_SW3 = 6
        self.ENC_READ_REG = 7
        self.SET_PID_P = 8
        self.SET_PID_I = 9
        self.SET_PID_D = 10
        self.SET_SPRING_CONSTANT = 11
        self.READ_POSITION = 12
        self.READ_CURRENT = 13
        self.READ_VELOCITY = 14
        self.SET_DAMPER_COEF = 15
        self.READ_WALL_ANGLE = 16


        self.divisor0 = 100
        self.dev = usb.core.find(idVendor=0x6666, idProduct=0x0003)
        if self.dev is None:
            raise ValueError(
                'no USB device found matching idVendor = 0x6666 and idProduct = 0x0003')
        self.dev.set_configuration()

# AS5048A Register Map
        self.ENC_NOP = 0x0000
        self.ENC_CLEAR_ERROR_FLAG = 0x0001
        self.ENC_PROGRAMMING_CTRL = 0x0003
        self.ENC_OTP_ZERO_POS_HI = 0x0016
        self.ENC_OTP_ZERO_POS_LO = 0x0017
        self.ENC_DIAG_AND_AUTO_GAIN_CTRL = 0x3FFD
        self.ENC_MAGNITUDE = 0x3FFE
        self.ENC_ANGLE_AFTER_ZERO_POS_ADDER = 0x3FFF

    def close(self):
        self.dev = None

    def toggle_led1(self):
        try:
            self.dev.ctrl_transfer(0x40, self.TOGGLE_LED1)
        except usb.core.USBError:
            print "Could not send TOGGLE_LED1 vendor request."

    def toggle_led2(self):
        try:
            self.dev.ctrl_transfer(0x40, self.TOGGLE_LED2)
        except usb.core.USBError:
            print "Could not send TOGGLE_LED2 vendor request."

    def toggle_led3(self):
        try:
            self.dev.ctrl_transfer(0x40, self.TOGGLE_LED3)
        except usb.core.USBError:
            print "Could not send TOGGLE_LED3 vendor request."

    def read_sw1(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.READ_SW1, 0, 0, 1)
        except usb.core.USBError:
            print "Could not send READ_SW1 vendor request."
        else:
            return int(ret[0])

    def read_sw2(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.READ_SW2, 0, 0, 1)
        except usb.core.USBError:
            print "Could not send READ_SW2 vendor request."
        else:
            return int(ret[0])

    def read_sw3(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.READ_SW3, 0, 0, 1)
        except usb.core.USBError:
            print "Could not send READ_SW3 vendor request."
        else:
            return int(ret[0])

    def set_pid_p(self, K):
        try:
            ret = self.dev.ctrl_transfer(
                0x40, self.SET_PID_P, int(K * self.divisor0), self.divisor0)
        except usb.core.USBError:
            print "Could not send SET_PID_P vendor request."

    def set_pid_i(self, K):
        try:
            ret = self.dev.ctrl_transfer(
                0x40, self.SET_PID_I, int(K * self.divisor0), self.divisor0)
        except usb.core.USBError:
            print "Could not send SET_PID_I vendor request."

    def set_pid_d(self, K):
        try:
            ret = self.dev.ctrl_transfer(
                0x40, self.SET_PID_D, int(K * self.divisor0), self.divisor0)
        except usb.core.USBError:
            print "Could not send SET_PID_D vendor request."

    def set_spring_constant(self, K):
        try:
            ret = self.dev.ctrl_transfer(
                0x40, self.SET_SPRING_CONSTANT, int(K * self.divisor0), self.divisor0)
        except usb.core.USBError:
            print "Could not send SET_SPRING_CONSTANT vendor request."

    def enc_readReg(self, address):
        try:
            ret = self.dev.ctrl_transfer(
                0xC0, self.ENC_READ_REG, address, 0, 2)
        except usb.core.USBError:
            print "Could not send ENC_READ_REG vendor request."
        else:
            return ret

    def read_position(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.READ_POSITION, 0, 0, 2)
        except usb.core.USBError, e:
            print e
            print "Could not send READ_POSITION vendor request."
        else:
            ret_value = int(ret[0]) + int(ret[1]) * 256
            return (ret_value/100.)- 50

    def read_velocity(self):
        try:
            ret = self.dev.ctrl_transfer(0xC0, self.READ_VELOCITY, 0, 0, 2)
        except usb.core.USBError, e:
            print e
            print "Could not send READ_POSITION vendor request."
        else:
            ret_value = int(ret[0]) + int(ret[1]) * 256
            return (ret_value/100.)- 50

    def set_damper_coef(self, K):
        try:
            ret = self.dev.ctrl_transfer(
                0x40, self.SET_DAMPER_COEF, int(K * self.divisor0), self.divisor0)
        except usb.core.USBError:
            print "Could not send SET_SPRING_CONSTANT vendor request."

    def set_wall_angle(self, wall_angle):
        try:
            ret = self.dev.ctrl_transfer(
                0x40, self.SET_DAMPER_COEF, int(wall_angle * self.divisor0), self.divisor0)
        except usb.core.USBError:
            print "Could not send SET_WALL_ANGLE vendor request."

    # def send_num(self, num):
    #     msg = 'test'
    #     msg = struct.pack('i', num)
    #     try:
    #         ret = self.dev.ctrl_transfer(
    #             0x40, self.SEND_NUM, 23, 223424,33)
    #     except usb.core.USBError:
    #         print "Could not send ENC_READ_REG vendor request."
    #     else:
    #         return ret
