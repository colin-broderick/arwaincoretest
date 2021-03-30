#!/usr/bin/python3
import sys
import time


class BMP280(object):

    ### BMP280 i2c address.
    I2CADDR = 0x76

    ### Operating modes.
    OSAMPLE_1 = 1
    OSAMPLE_2 = 2
    OSAMPLE_4 = 3
    OSAMPLE_8 = 4
    OSAMPLE_16 = 5

    ### Standby settings.
    STANDBY_0p5 = 0
    STANDBY_62p5 = 1
    STANDBY_125 = 2
    STANDBY_250 = 3
    STANDBY_500 = 4
    STANDBY_1000 = 5
    STANDBY_10 = 6
    STANDBY_20 = 7

    ### Filter configuration.
    FILTER_off = 0
    FILTER_2 = 1
    FILTER_4 = 2
    FILTER_8 = 3
    FILTER_16 = 4

    ### Registers.
    REGISTER_T1 = 0x88
    REGISTER_T2 = 0x8A
    REGISTER_T3 = 0x8C
    REGISTER_P1 = 0x8E
    REGISTER_P2 = 0x90
    REGISTER_P3 = 0x92
    REGISTER_P4 = 0x94
    REGISTER_P5 = 0x96
    REGISTER_P6 = 0x98
    REGISTER_P7 = 0x9A
    REGISTER_P8 = 0x9C
    REGISTER_P9 = 0x9E

    REGISTER_CHIPID = 0xD0
    REGISTER_VERSION = 0xD1
    REGISTER_SOFTRESET = 0xE0

    REGISTER_STATUS = 0xF3
    REGISTER_CONTROL = 0xF4
    REGISTER_CONFIG = 0xF5
    REGISTER_DATA = 0xF7


    def __init__(self, t_mode=OSAMPLE_1, p_mode=OSAMPLE_1, standby=STANDBY_250, filter=FILTER_off, address=I2CADDR, i2c=None, formula=1, **kwargs):
        ### Check that t_mode is valid.
        if t_mode not in [self.OSAMPLE_1, self.OSAMPLE_2, self.OSAMPLE_4, self.OSAMPLE_8, self.OSAMPLE_16]:
            raise ValueError('Unexpected t_mode value {0}.'.format(t_mode))
        self._t_mode = t_mode

        ### Check that p_mode is valid.
        if p_mode not in [self.OSAMPLE_1, self.OSAMPLE_2, self.OSAMPLE_4, self.OSAMPLE_8, self.OSAMPLE_16]:
            raise ValueError('Unexpected p_mode value {0}.'.format(p_mode))
        self._p_mode = p_mode

        ### Check that standby is valid.
        if standby not in [self.STANDBY_0p5, self.STANDBY_62p5, self.STANDBY_125, self.STANDBY_250, self.STANDBY_500, self.STANDBY_1000, self.STANDBY_10, self.STANDBY_20]:
            raise ValueError('Unexpected standby value {0}.'.format(standby))
        self._standby = standby

        ### Check that filter is valid.
        if filter not in [self.FILTER_off, self.FILTER_2, self.FILTER_4, self.FILTER_8, self.FILTER_16]:
            raise ValueError('Unexpected filter value {0}.'.format(filter))
        self._filter = filter

        ### Create I2C device.
        if i2c is None:
            import Adafruit_GPIO.I2C as I2C
            i2c = I2C

        ### Create device, catch permission errors
        try:
            self._device = i2c.get_i2c_device(address, **kwargs)
        except IOError:
            print("Unable to communicate with sensor, check permissions.")
            exit()

        ### Load calibration values.
        self.load_calibration()
        self._device.write8(self.REGISTER_CONTROL, 0x24)  # Sleep mode
        time.sleep(0.002)
        self._device.write8(self.REGISTER_CONFIG, ((standby << 5) | (filter << 2)))
        time.sleep(0.002)
        self._device.write8(self.REGISTER_CONTROL, ((t_mode << 5) | (p_mode << 2) | 3))  # Set Temp/Pressure Oversample and enter Normal mode
        self.t_fine = 0.0

        ### Choose which altitude formula to use based on command line argument.
        self.formula = formula

        ### Set initial values.
        self.sea_level_pressure = 1013.250
        self.temp = self.temperature
        self.p = self.pressure
        self.alt = self.altitude

    def load_calibration(self):
        ### Built-in calibration values. Can't be changed. Define your own values here if you don't like the manufacturer's.
        self.T1 = self._device.readU16LE(self.REGISTER_T1)
        self.T2 = self._device.readS16LE(self.REGISTER_T2)
        self.T3 = self._device.readS16LE(self.REGISTER_T3)
        self.P1 = self._device.readU16LE(self.REGISTER_P1)
        self.P2 = self._device.readS16LE(self.REGISTER_P2)
        self.P3 = self._device.readS16LE(self.REGISTER_P3)
        self.P4 = self._device.readS16LE(self.REGISTER_P4)
        self.P5 = self._device.readS16LE(self.REGISTER_P5)
        self.P6 = self._device.readS16LE(self.REGISTER_P6)
        self.P7 = self._device.readS16LE(self.REGISTER_P7)
        self.P8 = self._device.readS16LE(self.REGISTER_P8)
        self.P9 = self._device.readS16LE(self.REGISTER_P9)

    def read_temperature(self):
        ### Repeatedly check if data is ready, and raise error if it takes too long.
        for attempt in range(500):
            if self._device.readU8(self.REGISTER_STATUS) & 0x08:
                time.sleep(0.02)
            elif attempt == 499:
                raise IOError("Sensor took too long to respond.")
            else:
                break

        ### Once sensor is ready, read all relevant bytes.
        self.BMP280Data = self._device.readList(self.REGISTER_DATA, 6)
        temperature = ((self.BMP280Data[3] << 16) | (self.BMP280Data[4] << 8) | self.BMP280Data[5]) >> 4
        return temperature

    def read_pressure(self):
        ### Assumes read_temperature() already called, so just access the data array.
        pressure = ((self.BMP280Data[0] << 16) | (self.BMP280Data[1] << 8) | self.BMP280Data[2]) >> 4
        return pressure

    @property
    def temperature(self):
        UT = float(self.read_temperature())
        var1 = (UT / 16384.0 - float(self.T1) / 1024.0) * float(self.T2)
        var2 = ((UT / 131072.0 - float(self.T1) / 8192.0) * (
        UT / 131072.0 - float(self.T1) / 8192.0)) * float(self.T3)
        self.t_fine = int(var1 + var2)
        self.temp = (var1 + var2) / 5120.0
        return self.temp

    @property
    def pressure(self):
        adc = float(self.read_pressure())
        var1 = float(self.t_fine) / 2.0 - 64000.0
        var2 = var1 * var1 * float(self.P6) / 32768.0
        var2 = var2 + var1 * float(self.P5) * 2.0
        var2 = var2 / 4.0 + float(self.P4) * 65536.0
        var1 = (float(self.P3) * var1 * var1 / 524288.0 + float(self.P2) * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * float(self.P1)
        if var1 == 0:
            return 0
        p = 1048576.0 - adc
        p = ((p - var2 / 4096.0) * 6250.0) / var1
        var1 = float(self.P9) * p * p / 2147483648.0
        var2 = p * float(self.P8) / 32768.0
        self.p = p + (var1 + var2 + float(self.P7)) / 16.0
        return self.p

    @property
    def altitude(self):
        if self.formula == 1: return self.formula1()
        elif self.formula == 2: return self.formula2()
        else: return self.formula1

    def formula1(self):
        ### Hypsometric formula. Considers temperature. Only valid under 11 km.
        ### May not be valid in scenarios with high temperature but low pressure.
        self.alt = (((self.sea_level_pressure/(self.p/100))**(1/5.257)-1)*(self.temp+273.15))/0.0065
        return self.alt

    def formula2(self):
        ### Hypsometric formula but assumes constant temperature. Generally less accurate but that
        ### assumption may not hold in extreme temperatures.
        self.alt = 44330 * (1.0 - (self.p/100/self.sea_level_pressure)**0.1903)
        return self.alt
