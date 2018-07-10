# MicroPython-BME280

## <b>BME280: Digital Humidity, Pressure and Temperature Sensor</b>
The BME280 is a sensor from Bosch Sensortec for high precision measurements of Temperature, Pressure and Humidity
 <br>
### <b>DataSheet:</b>
https://www.mouser.com/datasheet/2/783/BST-BME280_DS001-11-844833.pdf <br>
or file <i>'BST-BME280_DS001-11-844833.pdf'</i><br>

## <b>Key features:</b>
<b>Package:</b>  &nbsp; 2.5 mm x 2.5 mm x 0.93 mm metal lid LGA<br>

<b>Digital interface: </b> &nbsp; I2C (up to 3.4 MHz) and SPI (3 and 4 wire, up to 10 MHz)<br>

<b>Supply voltage:</b>&nbsp; VDD main supply voltage range: 1.71 V to 3.6 V<br>
&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;VDD IO interface voltage range: 1.20 V to 3.6 V

<b>Current consumption:</b> 1.8 μA @ 1 Hz humidity and temperature
<br>&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; &nbsp; 2.8 μA @ 1 Hz pressure and temperature
<br>&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; &nbsp; 
3.6 μA @ 1 Hz humidity, pressure and temperature
<br>&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp;&emsp; &nbsp; 
0.1 μA in sleep mode<br>

<b>Operating range:</b>
-40...+85 °C, 0...100 % rel. humidity, 300...1100 hPa<br>

## <b>Files:</b>

<b>'bme280.py':</b> &nbsp;  Version for <i>Developers</i> of driver for geral devices compatibles of MicroPython - ESP8266, ESP32, LoPy, etc.<br>

<b>'bme280_lowmem.py':</b> &nbsp;  Version <i>Low Memory</i> of driver for geral devices compatibles of MicroPython - <b>No Documenteded</b><br>

<b>'bme280_microbit.py':</b> &nbsp;  Version for <i>Developers</i> of driver for BBC Micro:bit devices<br>

<b>'bme280_microbit_lowmem.py':</b> &nbsp;  Version <i>Low Memory</i> of driver for BBC Micro:bit devices - <b>No Documenteded</b><br>

## <b>Tests:</b>
#### ESP8266
```python

```
#### ESP32
```python

```
#### LoPy
```python
import bme280
from machine import I2C

i2c = I2C(0, I2C.MASTER, baudrate=100000)
sensor = bme280.BME280(i2c=i2c)

sensor.formated_values
```
#### BBC Micro:bit
```python

```
## <b>Driver Benchmark for Memory Consumed*:</b>
*To import all dependencies and construct the object.

|       |ESP8266|ESP32|LoPy|BBC Micro:bit|
|------:|:-----:|:---:|:--:|:-----------:|
|Memory (Bytes)|-|-|-|3568|
|Memory (%)|-|-|-|38|

The Test in BBC Micro:bit :
