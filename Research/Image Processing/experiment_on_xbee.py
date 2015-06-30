import numpy as np
import serial
import math

ser = serial.Serial(2)
angle = 50
while angle>0:
    ser.write("P")
    angle = angle-1
    print angle
