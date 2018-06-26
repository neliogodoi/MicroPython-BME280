import bme280_microbit_lowmem
from microbit import i2c
import gc
gc.collect()
gc.mem_free()
bme = bme280_microbit_lowmem.BME280(i2c)