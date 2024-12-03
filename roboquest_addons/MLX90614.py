#!/usr/bin/env python3

from time import sleep
from smbus2 import SMBus
from mlx90614 import MLX90614
import RPi.GPIO as GPIO

I2C_ENABLE_PIN = 17

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(I2C_ENABLE_PIN, GPIO.OUT)
GPIO.output(I2C_ENABLE_PIN, GPIO.HIGH)

bus = SMBus(6)
sensor = MLX90614(bus, address=0x5a)

for count in range(1, 1001):
    print(
        f'Ambient: {sensor.get_amb_temp():0.2f}'
        f', Object: {sensor.get_obj_temp():0.2f}'
    )
    sleep(0.5)

bus.close()
GPIO.output(I2C_ENABLE_PIN, GPIO.LOW)
GPIO.cleanup()
