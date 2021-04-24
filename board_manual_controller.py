import serial
import time
import cv2
 
aCenter = 82  # central position (default=90) of Servo A
bCenter = 85  # central position (default=90) of Servo B
d = 3  # resolution of servo movement
f = 66  # servo position (after calibration) that makes board flat
a = f
b = f

ser = serial.Serial('COM3',115200,timeout=1)
time.sleep(2)  # wait for the establishment of communication
# Nothing to display, but this enables us to use waitKey
cv2.namedWindow('frame')

ser.write(bytes([255,23,aCenter,bCenter,0]))
ser.write(bytes([255,24,a,0]))
ser.write(bytes([255,25,b,0]))

while (True):
    c = cv2.waitKey(1) & 0xFF
    if c == 255:
        continue
    """
    if c == ord('j'):
        a -= d; b += d
    elif c == ord('m'):
        a -= d; b -= d
    elif c == ord('i'):
        a += d; b += d
    elif c == ord('l'):
        a += d; b -= d
    elif c == ord('q'):
        break
    ser.write(bytes([255,24,a,0]))
    ser.write(bytes([255,25,b,0]))
    print("a = " + str(a) + ", b = " + str(b))
    time.sleep(0.1)
    """

    # overshoot for 100ms to get the ball rolling
    if c == ord('j'):
        a -= 2*d; b += 2*d
    elif c == ord('m'):
        a -= 2*d; b -= 2*d
    elif c == ord('i'):
        a += 2*d; b += 2*d
    elif c == ord('l'):
        a += 2*d; b -= 2*d
    elif c == ord('q'):
        break
    ser.write(bytes([255,24,a,0]))
    ser.write(bytes([255,25,b,0]))
    print("a = " + str(a) + ", b = " + str(b))
    time.sleep(0.1)

    # then move to the specified position
    if c == ord('j'):
        a -= -1*d; b += -1*d
    elif c == ord('m'):
        a -= -1*d; b -= -1*d
    elif c == ord('i'):
        a += -1*d; b += -1*d
    elif c == ord('l'):
        a += -1*d; b -= -1*d
    elif c == ord('q'):
        break
    ser.write(bytes([255,24,a,0]))
    ser.write(bytes([255,25,b,0]))
    print("a = " + str(a) + ", b = " + str(b))

ser.close()