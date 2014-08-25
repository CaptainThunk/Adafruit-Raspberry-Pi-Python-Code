#!/usr/bin/python

# Python library for Adafruit Flora Accelerometer/Compass Sensor (LSM303).
# This is pretty much a direct port of the current Arduino library and is
# similarly incomplete (e.g. no orientation value returned from read()
# method).  This does add optional high resolution mode to accelerometer
# though.

# Copyright 2013 Adafruit Industries

# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

# Modified for LSM303D

from Adafruit_I2C import Adafruit_I2C


class Adafruit_LSM303(Adafruit_I2C):

    # Minimal constants carried over from Arduino library
    LSM303_ADDRESS_ACCEL    = (0x1D)  # 0110101x
    L3G_GYRO                = (0x6B)  # 0001110x

    LSM303_CRA_REG_M         = 0x00 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_CRB_REG_M         = 0x01 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_MR_REG_M          = 0x02 # LSM303DLH, LSM303DLM, LSM303DLHC

    LSM303_TEMP_OUT_L        = 0x05 # LSM303D
    LSM303_TEMP_OUT_H        = 0x06 # LSM303D
    LSM303_STATUS_M          = 0x07 # LSM303D

    LSM303_SR_REG_M          = 0x09 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_IRA_REG_M         = 0x0A # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_IRB_REG_M         = 0x0B # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_IRC_REG_M         = 0x0C # LSM303DLH, LSM303DLM, LSM303DLHC

    LSM303_WHO_AM_I_M        = 0x0F # LSM303DLM
    LSM303_WHO_AM_I          = 0x0F # LSM303D

    LSM303_INT_CTRL_M        = 0x12 # LSM303D
    LSM303_INT_SRC_M         = 0x13 # LSM303D
    LSM303_INT_THS_L_M       = 0x14 # LSM303D
    LSM303_INT_THS_H_M       = 0x15 # LSM303D
    LSM303_OFFSET_X_L_M      = 0x16 # LSM303D
    LSM303_OFFSET_X_H_M      = 0x17 # LSM303D
    LSM303_OFFSET_Y_L_M      = 0x18 # LSM303D
    LSM303_OFFSET_Y_H_M      = 0x19 # LSM303D
    LSM303_OFFSET_Z_L_M      = 0x1A # LSM303D
    LSM303_OFFSET_Z_H_M      = 0x1B # LSM303D

    LSM303_REFERENCE_X       = 0x1C # LSM303D
    LSM303_REFERENCE_Y       = 0x1D # LSM303D
    LSM303_REFERENCE_Z       = 0x1E # LSM303D

    LSM303_CTRL0             = 0x1F # LSM303D
    LSM303_CTRL_REG1_A       = 0x20 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_CTRL1             = 0x20 # LSM303D
    LSM303_CTRL_REG2_A       = 0x21 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_CTRL2             = 0x21 # LSM303D
    LSM303_CTRL_REG3_A       = 0x22 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_CTRL3             = 0x22 # LSM303D
    LSM303_CTRL_REG4_A       = 0x23 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_CTRL4             = 0x23 # LSM303D
    LSM303_CTRL_REG5_A       = 0x24 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_CTRL5             = 0x24 # LSM303D
    LSM303_CTRL_REG6_A       = 0x25 # LSM303DLHC
    LSM303_CTRL6             = 0x25 # LSM303D
    LSM303_HP_FILTER_RESET_A = 0x25 # LSM303DLH, LSM303DLM
    LSM303_REFERENCE_A       = 0x26 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_CTRL7             = 0x26 # LSM303D
    LSM303_STATUS_REG_A      = 0x27 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_STATUS_A          = 0x27 # LSM303D

    LSM303_OUT_X_L_A         = 0x28 # LSM303DLH, LSM303DLM, LSM303DLHC, LSM303D
    LSM303_OUT_X_H_A         = 0x29 # LSM303DLH, LSM303DLM, LSM303DLHC, LSM303D
    LSM303_OUT_Y_L_A         = 0x2A # LSM303DLH, LSM303DLM, LSM303DLHC, LSM303D
    LSM303_OUT_Y_H_A         = 0x2B # LSM303DLH, LSM303DLM, LSM303DLHC, LSM303D
    LSM303_OUT_Z_L_A         = 0x2C # LSM303DLH, LSM303DLM, LSM303DLHC, LSM303D
    LSM303_OUT_Z_H_A         = 0x2D # LSM303DLH, LSM303DLM, LSM303DLHC, LSM303D

    LSM303_FIFO_CTRL_REG_A   = 0x2E # LSM303DLHC
    LSM303_FIFO_SRC_REG_A    = 0x2F # LSM303DLHC

    LSM303_INT1_CFG_A        = 0x30 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_IG_CFG1           = 0x30 # LSM303D
    LSM303_INT1_SRC_A        = 0x31 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_IG_SRC1           = 0x31 # LSM303D
    LSM303_INT1_THS_A        = 0x32 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_IG_THS1           = 0x32 # LSM303D
    LSM303_INT1_DURATION_A   = 0x33 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_IG_DUR1           = 0x33 # LSM303D
    LSM303_INT2_CFG_A        = 0x34 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_IG_CFG2           = 0x34 # LSM303D
    LSM303_INT2_SRC_A        = 0x35 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_IG_SRC2           = 0x35 # LSM303D
    LSM303_INT2_THS_A        = 0x36 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_IG_THS2           = 0x36 # LSM303D
    LSM303_INT2_DURATION_A   = 0x37 # LSM303DLH, LSM303DLM, LSM303DLHC
    LSM303_IG_DUR2           = 0x37 # LSM303D

    LSM303_CLICK_CFG_A       = 0x38 # LSM303DLHC
    LSM303_CLICK_CFG         = 0x38 # LSM303D
    LSM303_CLICK_SRC_A       = 0x39 # LSM303DLHC
    LSM303_CLICK_SRC         = 0x39 # LSM303D
    LSM303_CLICK_THS_A       = 0x3A # LSM303DLHC
    LSM303_CLICK_THS         = 0x3A # LSM303D
    LSM303_TIME_LIMIT_A      = 0x3B # LSM303DLHC
    LSM303_TIME_LIMIT        = 0x3B # LSM303D
    LSM303_TIME_LATENCY_A    = 0x3C # LSM303DLHC
    LSM303_TIME_LATENCY      = 0x3C # LSM303D
    LSM303_TIME_WINDOW_A     = 0x3D # LSM303DLHC
    LSM303_TIME_WINDOW       = 0x3D # LSM303D
    LSM303_ACT_THS           = 0x3E # LSM303D
    LSM303_ACT_DUR           = 0x3F # LSM303D

    LSM303_TEMP_OUT_H_M      = 0x31 # LSM303DLHC
    LSM303_TEMP_OUT_L_M      = 0x32 # LSM303DLHC

    # Dummy addresses for registers that have inconsistent addresses.
    LSM303_OUT_X_H_M         = -1
    LSM303_OUT_X_L_M         = -2
    LSM303_OUT_Y_H_M         = -3
    LSM303_OUT_Y_L_M         = -4
    LSM303_OUT_Z_H_M         = -5
    LSM303_OUT_Z_L_M         = -6

    # Specific addresses for the dummy addresses above:
    LSM303DLH_OUT_X_H_M      = 0x03
    LSM303DLH_OUT_X_L_M      = 0x04
    LSM303DLH_OUT_Y_H_M      = 0x05
    LSM303DLH_OUT_Y_L_M      = 0x06
    LSM303DLH_OUT_Z_H_M      = 0x07
    LSM303DLH_OUT_Z_L_M      = 0x08

    LSM303DLM_OUT_X_H_M      = 0x03
    LSM303DLM_OUT_X_L_M      = 0x04
    LSM303DLM_OUT_Z_H_M      = 0x05
    LSM303DLM_OUT_Z_L_M      = 0x06
    LSM303DLM_OUT_Y_H_M      = 0x07
    LSM303DLM_OUT_Y_L_M      = 0x08

    LSM303DLHC_OUT_X_H_M     = 0x03
    LSM303DLHC_OUT_X_L_M     = 0x04
    LSM303DLHC_OUT_Z_H_M     = 0x05
    LSM303DLHC_OUT_Z_L_M     = 0x06
    LSM303DLHC_OUT_Y_H_M     = 0x07
    LSM303DLHC_OUT_Y_L_M     = 0x08

    LSM303D_OUT_X_L_M        = 0x08
    LSM303D_OUT_X_H_M        = 0x09
    LSM303D_OUT_Y_L_M        = 0x0A
    LSM303D_OUT_Y_H_M        = 0x0B
    LSM303D_OUT_Z_L_M        = 0x0C
    LSM303D_OUT_Z_H_M        = 0x0D

    # L3G (gyro)
    L3G_WHO_AM_I      = 0x0F

    L3G_CTRL_REG1     = 0x20
    L3G_CTRL_REG2     = 0x21
    L3G_CTRL_REG3     = 0x22
    L3G_CTRL_REG4     = 0x23
    L3G_CTRL_REG5     = 0x24
    L3G_REFERENCE     = 0x25
    L3G_OUT_TEMP      = 0x26
    L3G_STATUS_REG    = 0x27

    L3G_OUT_X_L       = 0x28
    L3G_OUT_X_H       = 0x29
    L3G_OUT_Y_L       = 0x2A
    L3G_OUT_Y_H       = 0x2B
    L3G_OUT_Z_L       = 0x2C
    L3G_OUT_Z_H       = 0x2D

    L3G_FIFO_CTRL_REG = 0x2E
    L3G_FIFO_SRC_REG  = 0x2F

    L3G_INT1_CFG      = 0x30
    L3G_INT1_SRC      = 0x31
    L3G_INT1_THS_XH   = 0x32
    L3G_INT1_THS_XL   = 0x33
    L3G_INT1_THS_YH   = 0x34
    L3G_INT1_THS_YL   = 0x35
    L3G_INT1_THS_ZH   = 0x36
    L3G_INT1_THS_ZL   = 0x37
    L3G_INT1_DURATION = 0x38
    L3G_LOW_ODR       = 0x39

    def __init__(self, busnum=-1, debug=False, hires=False):

        # Accelerometer and altimeter are at different I2C
        # addresses, so invoke a separate I2C instance for each
        self.accel = Adafruit_I2C(self.LSM303_ADDRESS_ACCEL, busnum, debug)
        self.gyro  = Adafruit_I2C(self.L3G_GYRO, busnum, debug)

        ## LSM303D Accelerometer
        # AODR = 0101 (50 Hz ODR)
        # AZEN = AYEN = AXEN = 1 (all axes enabled)
        self.accel.write8(self.LSM303_CTRL1, 0x57)
        # AFS = 011 (8 g full scale)
        self.accel.write8(self.LSM303_CTRL2, 0x18)
  
        ## LSM303D Magnetometer
        # M_RES = 11 (high resolution mode)
        # M_ODR = 001 (6.25 Hz ODR)
        self.accel.write8(self.LSM303_CTRL5, 0xE4) # 0x64 = temp off / 0xE4 = temp on
        # MFS = 01 (4 gauss full scale)
        self.accel.write8(self.LSM303_CTRL6, 0x20)
        # MLP = 0 (low power mode off)
        # MD = 00 (continuous-conversion mode)
        self.accel.write8(self.LSM303_CTRL7, 0)
        
        # L3G gyro
        self.gyro.write8(self.L3G_CTRL_REG1, 0x0F)
        self.gyro.write8(self.L3G_CTRL_REG4, 0x20)

    # Interpret signed 12-bit acceleration component from list
    def accel12(self, list, idx):
        n = list[idx] | (list[idx+1] << 8) # Low, high bytes
        if n > 32767: n -= 65536           # 2's complement signed
        return n >> 4                      # 12-bit resolution

    # Interpret signed 16-bit magnetometer component from list
    def mag16(self, list, idx):
        n = (list[idx] << 8) | list[idx+1]   # High, low bytes
        return n if n < 32768 else n - 65536 # 2's complement signed

    def whoami(self):
        n = [(self.accel.readU8(self.LSM303_WHO_AM_I), self.gyro.readU8(self.L3G_WHO_AM_I))]
        
        return n

    # Interpret signed 16-bit 2's complement signed low, high
    def compliment2lh(self, list, idx):
        n = (list[idx]) | (list[idx+1] << 8)  # low, high bytes
        return n if n < 32768 else n - 65536 # 2's complement signed

    def read(self):
        
        

        # Read the magnetometer
        list = self.accel.readList(self.LSM303D_OUT_X_L_M | 0x80, 6)
        res = [(self.compliment2lh(list, 0),
                    self.compliment2lh(list, 2),
                    self.compliment2lh(list, 4),
                    0.0 )] # ToDo: Calculate orientation
        # Read the accelerometer
        list = self.accel.readList(self.LSM303_OUT_X_L_A | 0x80, 6)
        res.append(( self.compliment2lh(list, 0),
                 self.compliment2lh(list, 2),
                 self.compliment2lh(list, 4) ))
        # Read the gyro
        list = self.gyro.readList(self.L3G_OUT_X_L | 0x80, 6)
        res.append((self.compliment2lh(list, 0),
                    self.compliment2lh(list, 2),
                    self.compliment2lh(list, 4)))
        return res


    #def setMagGain(gain=LSM303_MAGGAIN_1_3):
    #    self.mag.write8( LSM303_REGISTER_MAG_CRB_REG_M, gain)

    def gettemp(self):
        list = self.accel.readList(self.LSM303_TEMP_OUT_L, 2)
        res = [(self.compliment2lh(list, 0))]
        return res

# Simple example prints accel/mag data once per second:
if __name__ == '__main__':

    from time import sleep

    lsm = Adafruit_LSM303()

    print '[(Magnetometer X, Y, Z, orientation), (Accelerometer X, Y, Z), (Gyro X, Y, Z)]'
    print lsm.whoami()
    print lsm.gettemp()
    while True:
        print lsm.read()
        sleep(1) # Output is fun to watch if this is commented out
