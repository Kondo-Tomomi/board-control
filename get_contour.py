import numpy as np
import cv2

# Print the HSV value of the pixel just clicked
def mouse_event(event, x, y, _, __):
    if event == cv2.EVENT_LBUTTONUP:
        pixel_value = frameHSV[y,x]
        print('(' + str(x) + ',' + str(y) + ') -> '+ str(pixel_value))

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
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    # Pick the region whose H(ue) value is within the ranges
    # Reddish colors may have H values close either to 0 or 180,
    # so we compose two masks below
    # Green and blue have H values close to 120 and 240, respectively
    mask1 = cv2.inRange(frameHSV, (0,100,60), (15,255,255))
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
            cv2.circle(frame, (cX, cY), 4, (127,255,255), -1)

    # Display the resulting frames
    cv2.imshow('webcam',frame)
    cv2.imshow('mask',mask)

    # Press 'q' in the frame window to exit loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
