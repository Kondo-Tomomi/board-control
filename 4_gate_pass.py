import serial
import time
import cv2
import numpy as np
import math

# Print the HSV value of the pixel just clicked
def mouse_event(event, x, y, _, __):
    if event == cv2.EVENT_LBUTTONUP:
        pixel_value = frameHSV[y,x]
        print('(' + str(x) + ',' + str(y) + ') -> '+ str(pixel_value))

def control(eX,eY,dt,preEX,preEY):
    # 20<=x<=300 22<=y<=215
    global iEX,iEY
    
    iEX += eX*dt
    iEY += eY*dt

    uX = (int)(Kp*eX + Ki*iEX + Kd*(eX-preEX)/dt)
    uY = (int)(Kp*eY + Ki*iEY + Kd*(eY-preEY)/dt)

    a = dx*uX; b = -dx*uX
    a -= dy*uY; b -= dy*uY

    # print("( uX, uY) = ("+str(uX)+","+str(uY),")")
    p=1; i=0
    while p>0 or i<5:
        p = 0
        if a>15:
            tmp = a/15
            b = int(b/tmp)
            a = 15
            p+=1
        if b>15:
            tmp = b/15
            a = int(a/tmp)
            b = 15
            p+=1
        if a<-15:
            tmp = -a/15
            b = int(b/tmp)
            a = -15
            p+=1
        if b<-15:
            tmp = -b/15
            a = int(a/tmp)
            b = -15
            p+=1
        i+=1

    ser.write(bytes([255,24,f+a,0]))
    ser.write(bytes([255,25,f+b,0]))

param = 3

def destination():
    global destinationX,destinationY,start,flag,xgate,ygate
    now = time.time()
    m = -1
    if (now-start)>1.5:
        start = now
        i = flag//param
        destinationY = ygate[i]
        if i%2 == 0:
            m = -1
        else:
            m = 1
        
        if (flag+1)%param == 0:
            destinationX = xgate[i]+ z*m
        else:
            destinationX = xgate[i]- z*m
        if i==2 and (flag+1)%param==1:
            destinationY = ygate[i-1]
        
        flag+=1
        if flag>=param*3:
            flag = 0

x1 = 24; x2 = 280; y1 = 25; y2=215
xgate = [80, 125, 220]
ygate = [160, 75, 130]

z = 25
aCenter = 80  # central position (default=90) of Servo A
bCenter = 83  # central position (default=90) of Servo B
dx = 3  # resolution of servo movement
dy = 4
Kp = 0.1; Ki = 0.0; Kd = 0.1

f = 66  # servo position (after calibration) that makes board flat
a = f
b = f
flag = 0

preEX = 100   # pre X - xt
iEX = 0.0       # integral
preEY = 100   # pre Y - yt
iEY = 0.0       # integral

destinationX = xgate[0] - z;  destinationY = ygate[0]

ser = serial.Serial('COM3',115200,timeout=1)
time.sleep(2)  # wait for the establishment of communication

ser.write(bytes([255,23,aCenter,bCenter,0]))
ser.write(bytes([255,24,a,0]))
ser.write(bytes([255,25,b,0]))

# Change the camera number as necessary
# Not that camera number may change by rebooting or re-connection!
cap = cv2.VideoCapture(1)

# Set frame size; for Logicool C270n, allowed values include
# 640x480, 320x240 and 160x120.  
# High-resolution images require more processing cost.
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320);
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240);

cv2.namedWindow('webcam', cv2.WINDOW_NORMAL)
cv2.setMouseCallback('webcam', mouse_event)
cv2.moveWindow('webcam',40,0)

# Another window for mask image
cv2.namedWindow('mask', cv2.WINDOW_NORMAL)
cv2.moveWindow('mask',480,0)

start = time.time()
preT = time.time()

while True:
    _, frame = cap.read()
    _, frame = cap.read()
    _, frame = cap.read()
    _, frame = cap.read()
    _, frame = cap.read()
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    # Pick the region whose H(ue) value is within the ranges
    # Reddish colors may have H values close either to 0 or 180,
    # so we compose two masks below
    # Green and blue have H values close to 120 and 240, respectively

    mask1 = cv2.inRange(frameHSV, (0,100,100), (15,255,255))
    mask2 = cv2.inRange(frameHSV, (170,100,50), (180,255,255))
    mask = mask1 + mask2

    # Find contours from the binary image
    # Result is an array of contour info; see OpenCV manual for details
    contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                        cv2.CHAIN_APPROX_SIMPLE)
    # Compute sizes (areas) of the contours found
    areas = [cv2.contourArea(c) for c in contours]

    # Draw the largest contour (if any) in green and its center in yellow
    if len(areas) > 0:
        max_index = np.argmax(areas)
        c = contours[max_index]
        cv2.drawContours(frame, [c], 0, (127,255,127), 2)

        # Compute the center of the contour; see OpenCV manual for details
        M = cv2.moments(c)
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            t = time.time()
            if preEX == 100 and preEY == 100:
                preEX = destinationX - cX
                preEY = destinationY - cY
            control(destinationX - cX,destinationY - cY, t - preT, preEX, preEY)
            preT = t; preEX = destinationX - cX; preEY = destinationY - cY
            cv2.circle(frame, (cX, cY), 4, (127,255,255), -1)
            cv2.circle(frame, (destinationX, destinationY), 4, (127,127,255), -1)
            destination()

    # Display the resulting frames
    cv2.imshow('webcam',frame)
    cv2.imshow('mask',mask)

    # Press 'q' in the frame window to exit loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

ser.close()
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()