# Untitled - By: timon - Fri Feb 20 2026

import sensor
import time
import pyb

uart = pyb.UART(1, 57600, timeout_char=50)
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

clock = time.clock()

def drive(motor_left: int, motor_right:int):
    uart.write(""+str(motor_left)+";"+str(motor_right)+"m")

print("FWD")
drive(100, 100)

time.sleep(5)

print("LEFT")
drive(-100, 100)

time.sleep(5)

print("RIGHT")
drive(100, -100)

time.sleep(5)

print("REV")
drive(-100, -100)

time.sleep(5)

print("STOP")
drive(0, 0)
