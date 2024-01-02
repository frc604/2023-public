# SPDX-FileCopyrightText: 2021 Kattni Rembor for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
Blink example for boards with ONLY a NeoPixel LED (e.g. without a built-in red LED).
Includes QT Py and various Trinkeys.
Requires two libraries from the Adafruit CircuitPython Library Bundle.
Download the bundle from circuitpython.org/libraries and copy the
following files to your CIRCUITPY/lib folder:
* neopixel.mpy
* adafruit_pixelbuf.mpy
Once the libraries are copied, save this file as code.py to your CIRCUITPY
drive to run it.

https://learn.adafruit.com/adafruit-qt-py-2040/connecting-to-the-serial-console
"""

import supervisor
import time
import board
import neopixel
import adafruit_vl53l4cd
import pwmio
import digitalio

MIN_DIST = 1.0  # cm
MAX_DIST = 100.0  # cm
DIST_RANGE = MAX_DIST - MIN_DIST
MIN_DUTY_CYCLE = 0.25
MAX_DUTY_CYCLE = 0.75
DUTY_CYCLE_RANGE = MAX_DUTY_CYCLE - MIN_DUTY_CYCLE
ERROR_DUTY_CYCLE = 0.1

# supervisor.disable_autoreload()

pixels = neopixel.NeoPixel(board.NEOPIXEL, 1)
i2c = board.STEMMA_I2C()
outPWM = pwmio.PWMOut(board.D0, frequency=100)

vl53 = adafruit_vl53l4cd.VL53L4CD(i2c)

# OPTIONAL: can set non-default values
vl53.inter_measurement = 0
vl53.timing_budget = 20

print("VL53L4CD Simple Test.")
print("--------------------")
model_id, module_type = vl53.model_info
print("Model ID: 0x{:0X}".format(model_id))
print("Module Type: 0x{:0X}".format(module_type))
print("Timing Budget: {}".format(vl53.timing_budget))
print("Inter-Measurement: {}".format(vl53.inter_measurement))
print("--------------------")

vl53.start_ranging()

while True:
    while not vl53.data_ready:
        pass
    vl53.clear_interrupt()
    distance = vl53.distance
    print("Distance: {} cm".format(distance))

    if distance > MIN_DIST and distance < MAX_DIST:
        pixels.fill((0, 127, 0))
        fraction = (distance - MIN_DIST) / DIST_RANGE
        duty_cycle = MIN_DUTY_CYCLE + fraction * DUTY_CYCLE_RANGE
        outPWM.duty_cycle = int(duty_cycle * (2**16 - 1))
    else:
        pixels.fill((127, 0, 0))
        outPWM.duty_cycle = int(ERROR_DUTY_CYCLE * (2**16 - 1))
