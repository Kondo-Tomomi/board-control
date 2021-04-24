import serial
import time
import cv2
import numpy as np

# Print the HSV value of the pixel just clicked
def mouse_event(event, x, y, _, __):
    if event == cv2.EVENT_LBUTTONUP:
        pixel_value = frameHSV[y,x]
        print('(' + str(x) + ',' + str(y) + ') -> '+ str(pixel_value))

# 66,54
# 66,75
# 72,60

def control(x,y):
    # 20<=x<=300 22<=y<=215
    if x<=160:
        if y>=160:
            ad = 2*d; bd = 0
        elif y<=70:
            ad = 0; bd = -2*d
        else:
            ad = d; bd = -d
    else:
        if y>=160:
            ad = 0; bd = 2*d
        elif y<=70:
            ad = -2*d; bd = 0
        else:
            ad = -d; bd = d
        
    print("ad = " + str(ad) + ", bd = " + str(bd))

    for i in [1,1,2,2,3,3,4,4,4,3,3,2,2,1,0]:
        ser.write(bytes([255,24,f+i*ad,0]))
        ser.write(bytes([255,25,f+i*bd,0]))
        time.sleep(0.02)
    time.sleep(1)
        

aCenter = 90  # central position (default=90) of Servo A
bCenter = 90  # central position (default=90) of Servo B
d = 3;  # resolution of servo movement
f = 66  # servo position (after calibration) that makes board flat
a = f;
b = f;
flag = 1;

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

while True:
    _, frame = cap.read()
    _, frame = cap.read()
    _, frame = cap.read()
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    # Pick the region whose H(ue) value is within the ranges
    # Reddish colors may have H values close either to 0 or 180,
    # so we compose two masks below
    # Green and blue have H values close to 120 and 240, respectively
    mask1 = cv2.inRange(frameHSV, (0,100,100), (15,255,255))
    mask2 = cv2.inRange(frameHSV, (170,100,100), (180,255,255))
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
            control(cX,cY)
            cv2.circle(frame, (cX, cY), 4, (127,255,255), -1)

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

