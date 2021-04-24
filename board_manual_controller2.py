import serial
import time
import cv2
 
aCenter = 88  # central position (default=90) of Servo A
bCenter = 88  # central position (default=90) of Servo B
d = 3   # resolution of servo movement
f = 66  # servo position (after calibration) that makes board flat

ser = serial.Serial('COM3',115200,timeout=1)
time.sleep(2)  # wait for the establishment of communication

# Nothing to display, but this enables us to use waitKey
cv2.namedWindow('frame')

ser.write(bytes([255,23,aCenter,bCenter,0]))
ser.write(bytes([255,24,f,0]))
ser.write(bytes([255,25,f,0]))

# h/j/k/l: roll ball to the west/south/north/east; q to quit
while (True):
    c = cv2.waitKey(1) & 0xFF
    ad = 0; bd = 0
    if c == ord('j'):
        ad = -d; bd = d
    elif c == ord('m'):
        ad = -d; bd = -d
    elif c == ord('i'):
        ad = d+2; bd = d+2
    elif c == ord('l'):
        ad = d; bd = -d
    elif c == ord('q'):
        break
    if ad != 0:
        for i in [1,2,3,4,5,5,4,4,3,3,2,2,1,1,0,0]:
            ser.write(bytes([255,24,f+i*ad,0]))
            ser.write(bytes([255,25,f+i*bd,0]))
            time.sleep(0.02)
ser.close()
