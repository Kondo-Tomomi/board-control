import serial
import time
import cv2
 
aCenter = 90  # central position (default=90) of Servo A
bCenter = 90  # central position (default=90) of Servo B
d = 1;  # resolution of servo movement
f = 66  # servo position (after calibration) that makes board flat
a = f;
b = f;

ser = serial.Serial('COM3',115200,timeout=1)
time.sleep(2)  # wait for the establishment of communication

ser.write(bytes([255,23,aCenter,bCenter,0]))
a=75; b=66;

ser.write(bytes([255,24,a,0]))
ser.write(bytes([255,25,b,0]))

ser.close()