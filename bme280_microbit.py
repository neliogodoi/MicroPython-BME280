"""
MicroPython driver for BME280 Temperature, Pressure and Humidity, developer version, specific for BBC Micro:bit :
https://github.com/neliogodoi/MicroPython-BME280
Authors: Nelio Goncalves Godoi 2017, Roberto Colistete Jr 2017
Based on the work by authors Paul Cunnane 2016, Peter Dahlebrg 2016
Version: 0.1.0 @ 2018/06/21
"""

from microbit import sleep


class BME280:
    """
    Class Driver for BME280 Temperature, Pressure and Humidity digital sensor
    """
    
    def __init__(self,
                 i2c,          # I2C Object
                 temp_mode=2,  # mode of operation for Temperature sensor
                 pres_mode=5,  # mode of operation for Pressure sensor
                 humi_mode=1,  # mode of operation for Humidity sensor
                 iir=4,        # LOW PASS IIR FILTER SETTING
                 address=0x77):# I2C address of sensor

        # For default values x2 temperature, x16 pressure and
        # x1 humidity oversampling, IIR filter x16 :
        # Maximum measurement time =  t_measured,
        # max= ((1.25 + (2.3*2)+(2.3*16 + 0.575)+(2.3*1 + 0.575))ms = 46,1ms
        # Filter bandwidth response time = 22 samples
        # to reach 75% of step response x 46,1 ms = 1.0142 s

        self.temp_mode = temp_mode
        self.pres_mode = pres_mode
        self.humi_mode = humi_mode
        self.iir = iir
        self.address = address
        self._i2c = i2c
        self._t_fine = 0

        # load calibration data
        self.dig_T1 = self._read16(0x88)
        self.dig_T2 = self._short(self._read16(0x8A))
        self.dig_T3 = self._short(self._read16(0x8C))
        self.dig_P1 = self._read16(0x8E)
        self.dig_P2 = self._short(self._read16(0x90))
        self.dig_P3 = self._short(self._read16(0x92))
        self.dig_P4 = self._short(self._read16(0x94))
        self.dig_P5 = self._short(self._read16(0x96))
        self.dig_P6 = self._short(self._read16(0x98))
        self.dig_P7 = self._short(self._read16(0x9A))
        self.dig_P8 = self._short(self._read16(0x9C))
        self.dig_P9 = self._short(self._read16(0x9E))
        self.dig_H1 = self._read8(0xA1)
        self.dig_H2 = self._short(self._read16(0xE1))
        self.dig_H3 = self._read8(0xE3)
        a = self._read8(0xE5)
        self.dig_H4 = (self._read8(0xE4) << 4) + (a % 16)
        self.dig_H5 = (self._read8(0xE6) << 4) + (a >> 4)
        self.dig_H6 = self._read8(0xE7)
        if self.dig_H6 > 127:
            self.dig_H6 -= 256
        # Configure oversampling of humidity sensor
        self._write8(0xF2, self.humi_mode)
        sleep(0.002)
        self._write8(0xF4, 0x24) # 0x24 normal mode operation 
                                 # 0x00 sleep  mode operation
        # Sleep mode with temperature and pressure with 1x oversampling
        sleep(0.002)
        # Configure low pass IIR filter
        self._write8(0xF5, self.iir << 2)

    def _read8(self, reg):
        """
        Reads one bytes from the sensor
        """
        self._i2c.write(self.address, bytearray([reg]))
        t = self._i2c.read(self.address, 1)
        return t[0]

    def _read16(self, reg):
        """
        Reads two byte from the sensor
        """
        self._i2c.write(self.address, bytearray([reg]))
        t = self._i2c.read(self.address, 2)
        return t[0]+t[1]*256

    def _write8(self, reg, dat):
        """
        Write one byte in sensor
        """
        self._i2c.write(self.address, bytearray([reg, dat]))

    def _short(self, dat):
        return [dat-65536, dat][dat > 32767] #if-else

    def read_raw_data(self):
        """
        Reads raw data from the sensor
        """
        # Configure oversampling of pressure and temperature sensors,
        # enter forced mode (needed for every reading)
        self._write8(0xF4, (self.pres_mode << 5 | self.temp_mode << 2 | 1))

        sleep_time = 1250
        if self.temp_mode in [1, 2, 3, 4, 5]:
            sleep_time += 2300*(1 << self.temp_mode)
        if self.pres_mode in [1, 2, 3, 4, 5]:
            sleep_time += 575 + (2300*(1 << self.pres_mode))
        if self.humi_mode in [1, 2, 3, 4, 5]:
            sleep_time += 575 + (2300*(1 << self.humi_mode))
        # Wait the required for max measurement time
        sleep(sleep_time/1000000) 

        while(self._read16(0xF3) & 0x08):
            sleep(0.001)
        # burst readout from 0xF7 to 0xFE, recommended by datasheet
        # pressure(0xF7): ((msb << 16) | (lsb << 8) | xlsb) >> 4
        raw_pres = ((self._read8(0xF7)<<16) | (self._read8(0xF8)<<8) | self._read8(0xF9))>>4
        # temperature(0xFA): ((msb << 16) | (lsb << 8) | xlsb) >> 4
        raw_temp = ((self._read8(0xFA)<<16) | (self._read8(0xFB)<<8) | self._read8(0xFC))>>4
        # humidity(0xFD): (msb << 8) | lsb
        raw_humi = (self._read8(0xFD) << 8) | self._read8(0xFE)

        return (raw_temp, raw_pres, raw_humi)

    def read_compensated_data(self):
        """
        Compensates raw sensor data
        """
        raw_temp, raw_press, raw_hum = self.read_raw_data()
        # Temperature
        var1 = ((raw_temp >> 3) - (self.dig_T1 << 1)) * (self.dig_T2 >> 11)
        var2 = (raw_temp >> 4) - self.dig_T1
        var2 = var2 * ((raw_temp >> 4) - self.dig_T1)
        var2 = ((var2 >> 12) * self.dig_T3) >> 14

        self._t_fine = var1 + var2
        temp = (self._t_fine * 5 + 128) >> 8
        # Pressure
        var1 = self._t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 17)
        var2 = var2 + (self.dig_P4 << 35)
        var1 = (((var1 * var1 * self.dig_P3) >> 8) +
                ((var1 * self.dig_P2) << 12))
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33
        if var1 == 0:
            pres = 0
        else:
            p = 1048576 - raw_press
            p = (((p << 31) - var2) * 3125) // var1
            var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
            var2 = (self.dig_P8 * p) >> 19
            pres = ((p + var1 + var2) >> 8) + (self.dig_P7 << 4)
        # Humidity
        h = self._t_fine - 76800
        h = (((((raw_hum << 14) - (self.dig_H4 << 20) -
                (self.dig_H5 * h)) + 16384)
              >> 15) * (((((((h * self.dig_H6) >> 10) *
                            (((h * self.dig_H3) >> 11) + 32768)) >> 10) +
                          2097152) * self.dig_H2 + 8192) >> 14))
        h = h - (((((h >> 15) * (h >> 15)) >> 7) * self.dig_H1) >> 4)
        h = 0 if h < 0 else h
        h = 419430400 if h > 419430400 else h
        humi = h >> 12

        return (temp, pres, humi)

    def values(self):
        """
        Returns 3 double values for Temperature, Pressure and Humidity in this order
        """
        temp, pres, humi = self.read_compensated_data()
        return (temp/100, pres/256,  humi/1024)

    def pressure_precision(self):
        """
        Returns 2 double values for integer and decimal parts of Pressure in this order
        """
        p = self.read_compensated_data()[1]
        pi = float(p // 256)
        pd = (p % 256)/256
        return (pi, pd)

    def altitude(self, pressure_sea_level=1013.25):
        """
        Returns estimated altitude based on International Barometric Formula
        """
        # The International Barometric Formula using hPa.
        pi, pd = self.pressure_precision()
        return 44330*(1-((float(pi+pd)/100)/pressure_sea_level)**(1/5.255))
