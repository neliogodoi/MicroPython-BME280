from microbit import i2c
import gc
gc.collect()
import bme280_microbit_lowmem
bme = bme280_microbit_lowmem.BME280(i2c)