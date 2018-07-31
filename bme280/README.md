# BME280 ESP32 I2C ESP-IDF Example.

This example uses the official BOSCH BME280 Library to interface with BME280.
Only one line has been changed in the BOSCH Library - line 105 in bme280_defs.h
from /* #define BME280_FLOAT_ENABLE */ --> #define BME280_FLOAT_ENABLE
This is done to enable float ouput.
