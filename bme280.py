"""
Authors: Nelio Goncalves Godoi 2017, Roberto Colistete Jr 2017
Last update: 31/03/2018

Based on the work by authors Paul Cunnane 2016, Peter Dahlebrg 2016

Added :
- diferent oversampling modes for temperature, pressure and humidity.
  Default values are x2, x16 and x1, respectively;
- low pass IIR filter settings, when enabled the temperature and pressure
  resolutions are 20 bits, default value is x16;
- 3 temperature scales : Celsius, Fahrenheit, Kelvin;
- wait for sensor data conversion to complete before reading;
- output format "values" returning 3 floats, with pressure in Pa;
- output format "formatedValues" returning 3 strings, with pressure in Pa;
- output format "temperature" returning as a float number;
- output format "pressure" returning as a float number in Pa;
- output format "pressurePrecision" returning 2 float numbers, the integer
  part and decimal part in Pa;
- output format "humidity" returning as a float number.

31/03/2018

- function "altitude(pressure)" output format "altitude" returning as a float number.
"""

import time
from ustruct import unpack, unpack_from
from array import array

# BME280 default address
BME280_I2CADDR = 0x76
# BME280_I2CADDR = 0x77

# OPERATING AND OVERSAMPLING MODES
OSAMPLE_0 = 0    # Sensor is skipped (output set to 0x80000)
OSAMPLE_1 = 1
OSAMPLE_2 = 2
OSAMPLE_4 = 3
OSAMPLE_8 = 4
OSAMPLE_16 = 5

# BME280 REGISTERS
BME280_REGISTER_STATUS = 0xF3        # Status of sensor data
BME280_REGISTER_CONTROL_HUM = 0xF2   # Humidity
BME280_REGISTER_CONTROL = 0xF4       # Temperature and pressure
BME280_REGISTER_CONTROL_IIR = 0xF5   # Low pass IIR filter

# LOW PASS IIR FILTER SETTINGS
FILTER_OFF = 0
FILTER_2 = 1
FILTER_4 = 2
FILTER_8 = 3
FILTER_16 = 4

# TEMPERATURE SCALES/UNITS
CELSIUS = 'C'
FAHRENHEIT = 'F'
KELVIN = 'K'

class BME280(object):

    """ Driver for BME280 sensor of Temperature, Pressure and Humidity """
    # For default values x2 temperature, x16 pressure and
    # x1 humidity oversampling, IIR filter x16 :
    # Maximum measurement time =  t_measured,
    # max = ((1.25 + (2.3*2) + (2.3*16 + 0.575) + (2.3*1 + 0.575)) ms = 46,1 ms
    # Filter bandwidth response time = 22 samples
    # to reach 75% of step response x 46,1 ms = 1.0142 s

    def __init__(self,
                 temperature_mode=OSAMPLE_2,
                 pressure_mode=OSAMPLE_16,
                 humidity_mode=OSAMPLE_1,
                 temperature_scale=CELSIUS,
                 iir=FILTER_16,
                 address=BME280_I2CADDR,
                 i2c):

        osamples = [
            OSAMPLE_0,
            OSAMPLE_1,
            OSAMPLE_2,
            OSAMPLE_4,
            OSAMPLE_8,
            OSAMPLE_16]

        msg_error = 'Unexpected {} operating mode value {0}.'

        # Check that temperature_mode is valid
        if temperature_mode not in osamples:
            raise ValueError(msg_error.format("temperature", temperature_mode))
        self.temperature_mode = temperature_mode

        # Check that pressure_mode is valid
        if pressure_mode not in osamples:
            raise ValueError(msg_error.format("pressure", pressure_mode))
        self.pressure_mode = pressure_mode

        # Check that humidity_mode is valid
        if humidity_mode not in osamples:
            raise ValueError(msg_error.format("humidity", humidity_mode))
        self.humidity_mode = humidity_mode

        msg_error = 'Unexpected low pass IIR filter setting value {0}.'

        # Check that low pass IIR filter setting is valid
        if iir not in [FILTER_OFF, FILTER_2, FILTER_4, FILTER_8, FILTER_16]:
            raise ValueError(msg_error.format(iir))
        self.iir = iir

        msg_error = 'Unexpected temperature scale value {0}.'

        # Check that temperature scale valid
        if temperature_scale not in [CELSIUS, FAHRENHEIT, KELVIN]:
            raise ValueError(msg_error.format(temperature_scale))
        self.temperature_scale = temperature_scale

        del msg_error

        self.address = address
        self.i2c = i2c

        # load calibration data
        dig_88_a1 = self.i2c.readfrom_mem(self.address, 0x88, 26)
        dig_e1_e7 = self.i2c.readfrom_mem(self.address, 0xE1, 7)
        self.dig_T1, self.dig_T2, self.dig_T3, self.dig_P1, \
            self.dig_P2, self.dig_P3, self.dig_P4, self.dig_P5, \
            self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9, \
            _, self.dig_H1 = unpack("<HhhHhhhhhhhhBB", dig_88_a1)

        self.dig_H2, self.dig_H3 = unpack("<hB", dig_e1_e7)
        e4_sign = unpack_from("<b", dig_e1_e7, 3)[0]
        self.dig_H4 = (e4_sign << 4) | (dig_e1_e7[4] & 0xF)

        e6_sign = unpack_from("<b", dig_e1_e7, 5)[0]
        self.dig_H5 = (e6_sign << 4) | (dig_e1_e7[4] >> 4)

        self.dig_H6 = unpack_from("<b", dig_e1_e7, 6)[0]

        self.i2c.writeto_mem(
            self.address,
            BME280_REGISTER_CONTROL,
            bytearray([0x24]))
        # Sleep mode with temperature and pressure with 1x oversampling
        time.sleep(0.002)   # Needed ?

        self.t_fine = 0
        # temporary data holders which stay allocated
        self._l1_barray = bytearray(1)
        self._l8_barray = bytearray(8)
        self._l3_resultarray = array("i", [0, 0, 0])

        # Configure low pass IIR filter
        self._l1_barray[0] = self.iir << 2
        self.i2c.writeto_mem(
            self.address,
            BME280_REGISTER_CONTROL_IIR,
            self._l1_barray)
        time.sleep(0.002)   # Needed ?

        # Configure oversampling of humidity sensor
        self._l1_barray[0] = self.humidity_mode
        self.i2c.writeto_mem(
            self.address,
            BME280_REGISTER_CONTROL_HUM,
            self._l1_barray)

    def read_raw_data(self, result):
        """ Read the raw data of the sensor hardware """
        # Configure oversampling of pressure and temperature sensors,
        # enter forced mode (needed for every reading)
        self._l1_barray[0] = (
            self.pressure_mode << 5 |
            self.temperature_mode << 2 | 1)

        self.i2c.writeto_mem(
            self.address,
            BME280_REGISTER_CONTROL,
            self._l1_barray)

        osamples_1_16 = [
            OSAMPLE_1,
            OSAMPLE_2,
            OSAMPLE_4,
            OSAMPLE_8,
            OSAMPLE_16]

        sleep_time = 1250
        if self.temperature_mode in osamples_1_16:
            sleep_time += 2300*(1 << self.temperature_mode)
        if self.pressure_mode in osamples_1_16:
            sleep_time += 575 + (2300*(1 << self.pressure_mode))
        if self.humidity_mode in osamples_1_16:
            sleep_time += 575 + (2300*(1 << self.humidity_mode))
        time.sleep_us(sleep_time)  # Wait the required for max measurement time

        # Wait for sensor data conversion to complete
        while (unpack('<H',
                      self.i2c.readfrom_mem(
                          self.address,
                          BME280_REGISTER_STATUS, 2))[0] & 0x08):
            time.sleep(0.001)

        # burst readout from 0xF7 to 0xFE, recommended by datasheet
        self.i2c.readfrom_mem_into(self.address, 0xF7, self._l8_barray)
        readout = self._l8_barray
        # pressure(0xF7): ((msb << 16) | (lsb << 8) | xlsb) >> 4
        raw_press = ((readout[0] << 16) | (readout[1] << 8) | readout[2]) >> 4
        # temperature(0xFA): ((msb << 16) | (lsb << 8) | xlsb) >> 4
        raw_temp = ((readout[3] << 16) | (readout[4] << 8) | readout[5]) >> 4
        # humidity(0xFD): (msb << 8) | lsb
        raw_hum = (readout[6] << 8) | readout[7]

        result[0] = raw_temp
        result[1] = raw_press
        result[2] = raw_hum

    def read_compensated_data(self, result=None):
        """ Get raw data and compensa the same """
        self.read_raw_data(self._l3_resultarray)
        raw_temp, raw_press, raw_hum = self._l3_resultarray
        # temperature
        var1 = ((raw_temp >> 3) - (self.dig_T1 << 1)) * (self.dig_T2 >> 11)

        var2 = (raw_temp >> 4) - self.dig_T1
        var2 = var2 * ((raw_temp >> 4) - self.dig_T1)
        var2 = ((var2 >> 12) * self.dig_T3) >> 14

        self.t_fine = var1 + var2
        temp = (self.t_fine * 5 + 128) >> 8

        # pressure
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

        # humidity
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
        """ return 3 floats numbers with values of temperature (C or F or K),
        pressure (Pa), humidity (%) """
        temp, pres, humi = self.read_compensated_data()
        temp = temp/100              # CELCIUS
        if self.temperature_scale == 'F':
            temp = 32 + (temp*1.8)   # FAHRENHEIT
        elif self.temperature_scale == 'K':
            temp = temp + 273.15     # KELVIN

        pres = pres/256              # Pa
        humi = humi/1024             # %
        return (temp, pres, humi)

    @property
    def formated_values(self):
        """ human readable values """
        t, p, h = self.values
        temp = "{} "+self.temperature_scale
        return (temp.format(t), "{} Pa".format(p), "{} %".format(h))

    @property
    def temperature(self):
        """ Return the temperature value """
        t, _, _ = self.values
        return t

    @property
    def pressure(self):
        """ Return the pressure value """
        _, p, _ = self.values
        return p

    @property
    def pressure_precision(self):
        """ Return the pressure integer and decimal values"""
        _, p, _ = self.read_compensated_data()
        pi = float(p // 256)
        pd = (p % 256)/256
        return (pi, pd)

    @property
    def humidity(self):
        """ Return the humidity value """
        _, _, h = self.values
        return h

    def altitude(self, pressure_sea_level=1013.25):
        """ Returns the estimated altitude value"""
        # The International Barometric Formula using hPa.
        _, p, _ = self.values
        return 44330*(1-((p/100)/pressure_sea_level)**(1/5.255))
