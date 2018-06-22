"""
MicroPython driver for BME280 Temperature, Pressure and Humidity, developer version, specific for BBC Micro:bit :
https://github.com/neliogodoi/MicroPython-BME280
Version: 0.1.0 @ 2018/06/21
"""

from time import sleep, sleep_us
from ustruct import unpack, unpack_from
from array import array

BME280_I2CADDR = 0x76

class BME280(object):

    def __init__(self,
                 temp_mode=2,
                 pres_mode=5,
                 humi_mode=1,
                 temp_scale='C',
                 iir=4,
                 address=BME280_I2CADDR,
                 i2c):
        osamples = [0, 1, 2, 3, 4, 5]
        msg_error = 'Unexpected {} operating mode value {0}.'
        if temp_mode not in osamples:
            raise ValueError(msg_error.format("temperature", temp_mode))
        self.temp_mode = temp_mode
        if pres_mode not in osamples:
            raise ValueError(msg_error.format("pressure", pres_mode))
        self.pres_mode = pres_mode
        if humi_mode not in osamples:
            raise ValueError(msg_error.format("humidity", humi_mode))
        self.humi_mode = humi_mode
        msg_error = 'Unexpected low pass IIR filter setting value {0}.'
        if iir not in [0, 1, 2, 3, 4]:
            raise ValueError(msg_error.format(iir))
        self.iir = iir
        msg_error = 'Unexpected temperature scale value {0}.'
        if temp_scale not in ['C', 'F', 'K']:
            raise ValueError(msg_error.format(temp_scale))
        self.temp_scale = temp_scale
        del msg_error
        self.address = address
        self.i2c = i2c
        dig_88_a1 = self.i2c.readfrom_mem(self.address, 0x88, 26)
        dig_e1_e7 = self.i2c.readfrom_mem(self.address, 0xE1, 7)
        self.dig_T1, \
        self.dig_T2, \
        self.dig_T3, \
        self.dig_P1, \
        self.dig_P2, \
        self.dig_P3, \
        self.dig_P4, \
        self.dig_P5, \
        self.dig_P6, \
        self.dig_P7, \
        self.dig_P8, \
        self.dig_P9, \
        _, \
        self.dig_H1 = unpack("<HhhHhhhhhhhhBB", dig_88_a1)
        self.dig_H2, self.dig_H3 = unpack("<hB", dig_e1_e7)
        e4_sign = unpack_from("<b", dig_e1_e7, 3)[0]
        self.dig_H4 = (e4_sign << 4) | (dig_e1_e7[4] & 0xF)
        e6_sign = unpack_from("<b", dig_e1_e7, 5)[0]
        self.dig_H5 = (e6_sign << 4) | (dig_e1_e7[4] >> 4)
        self.dig_H6 = unpack_from("<b", dig_e1_e7, 6)[0]
        self.i2c.writeto_mem(
            self.address,
            0xF4,
            bytearray([0x24]))
        time.sleep(0.002)
        self.t_fine = 0
        self._l1_barray = bytearray(1)
        self._l8_barray = bytearray(8)
        self._l3_resultarray = array("i", [0, 0, 0])
        self._l1_barray[0] = self.iir << 2
        self.i2c.writeto_mem(self.address, 0xF5, self._l1_barray)
        time.sleep(0.002)
        self._l1_barray[0] = self.humi_mode
        self.i2c.writeto_mem(self.address, 0xF2, self._l1_barray)

    def read_raw_data(self, result):
        self._l1_barray[0] = (self.pres_mode << 5 | self.temp_mode << 2 | 1)
        self.i2c.writeto_mem(self.address, 0xF4, self._l1_barray)
        osamples_1_16 = [1, 2, 3, 4, 5]
        sleep_time = 1250
        if self.temp_mode in osamples_1_16:
            sleep_time += 2300*(1 << self.temp_mode)
        if self.pres_mode in osamples_1_16:
            sleep_time += 575 + (2300*(1 << self.pres_mode))
        if self.humi_mode in osamples_1_16:
            sleep_time += 575 + (2300*(1 << self.humi_mode))
        time.sleep_us(sleep_time)
        while (unpack('<H', self.i2c.readfrom_mem(
                                self.address, 0xF3, 2))[0] & 0x08):
            time.sleep(0.001)
        self.i2c.readfrom_mem_into(self.address, 0xF7, self._l8_barray)
        readout = self._l8_barray
        raw_press = ((readout[0] << 16) | (readout[1] << 8) | readout[2]) >> 4
        raw_temp = ((readout[3] << 16) | (readout[4] << 8) | readout[5]) >> 4
        raw_hum = (readout[6] << 8) | readout[7]
        result[0] = raw_temp
        result[1] = raw_press
        result[2] = raw_hum

    def read_compensated_data(self, result=None):
        self.read_raw_data(self._l3_resultarray)
        raw_temp, raw_press, raw_hum = self._l3_resultarray
        var1 = ((raw_temp >> 3) - (self.dig_T1 << 1)) * (self.dig_T2 >> 11)
        var2 = (raw_temp >> 4) - self.dig_T1
        var2 = var2 * ((raw_temp >> 4) - self.dig_T1)
        var2 = ((var2 >> 12) * self.dig_T3) >> 14
        self.t_fine = var1 + var2
        temp = (self.t_fine * 5 + 128) >> 8
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 17)
        var2 = var2 + (self.dig_P4 << 35)
        var1 = (((var1 * var1 * self.dig_P3) >> 8) +
                ((var1 * self.dig_P2) << 12))
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33
        if var1 == 0:
            pressure = 0
        else:
            p = 1048576 - raw_press
            p = (((p << 31) - var2) * 3125) // var1
            var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
            var2 = (self.dig_P8 * p) >> 19
            pressure = ((p + var1 + var2) >> 8) + (self.dig_P7 << 4)
        h = self.t_fine - 76800
        h = (((((raw_hum << 14) - (self.dig_H4 << 20) -
                (self.dig_H5 * h)) + 16384)
              >> 15) * (((((((h * self.dig_H6) >> 10) *
                            (((h * self.dig_H3) >> 11) + 32768)) >> 10) +
                          2097152) * self.dig_H2 + 8192) >> 14))
        h = h - (((((h >> 15) * (h >> 15)) >> 7) * self.dig_H1) >> 4)
        h = 0 if h < 0 else h
        h = 419430400 if h > 419430400 else h
        humidity = h >> 12
        if result:
            result[0] = temp
            result[1] = pressure
            result[2] = humidity
            return result
        return array("i", (temp, pressure, humidity))

    @property
    def values(self):
        temp, pres, humi = self.read_compensated_data()
        temp = temp/100
        if self.temp_scale == 'F':
            temp = 32 + (temp*1.8)
        elif self.temp_scale == 'K':
            temp = temp + 273.15
        pres = pres/256
        humi = humi/1024
        return (temp, pres, humi)

    @property
    def formated_values(self):
        t, p, h = self.values
        temp = "{} "+self.temp_scale
        return (temp.format(t), "{} Pa".format(p), "{} %".format(h))

    @property
    def temperature(self):
        t, _, _ = self.values
        return t

    @property
    def pressure(self):
        _, p, _ = self.values
        return p

    @property
    def pressure_precision(self):
        _, p, _ = self.read_compensated_data()
        pi = float(p // 256)
        pd = (p % 256)/256
        return (pi, pd)

    @property
    def humidity(self):
        _, _, h = self.values
        return h

    def altitude(self, pressure_sea_level=1013.25):
        pi, pd = self.pressure_precision
        return 44330*(1-((float(pi+pd)/100)/pressure_sea_level)**(1/5.255))
