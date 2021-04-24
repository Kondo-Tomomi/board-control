# Simple servo swing program 
import serial
import time
 
# Port name can be found in Arduino IDE (under Tools menu)
# ser = serial.Serial('/dev/ttyACM0',115200,timeout=1)
ser = serial.Serial('COM3',115200,timeout=1)

time.sleep(2)  # wait for the establishment of communication

# Set the central angles of the two servos
ser.write(bytes([255,23,88,88,0]))

# To show two useful ways to specify for loops
while True:
    for i in [30,35,40,45,50,55,60,65,70,75,80,85,90]:
        ser.write(bytes([255,24,i,0]))
        ser.write(bytes([255,25,i,0]))
        time.sleep(0.15)
    for i in range(90,30,-1):
        ser.write(bytes([255,24,i,0]))
        ser.write(bytes([255,25,i,0]))
        time.sleep(0.03)

ser.close()
