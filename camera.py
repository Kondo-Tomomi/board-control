import numpy as np
import cv2

# Change the camera number as necessary
# Not that camera number may change by rebooting or re-connection!
cap = cv2.VideoCapture(1)

# Create a named window so that properties (e.g., window position)
# can be specified later
cv2.namedWindow('webcam', cv2.WINDOW_AUTOSIZE)

# Set frame size; for Logicool C270n, possible values include
# 640x480, 320x240 and 160x120.  
# High-resolution images require more processing cost.
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320);
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240);

while True:
    # Capture frame-by-frame; the first return value is not used
    _, frame = cap.read()

    # Operations on the frame come here
    # Note that the frame is in the BGR (not RGB) format!
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('webcam', gray)

    # Press 'q' in the frame window to exit loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything has been done, release the capture
cap.release()
cv2.destroyAllWindows()
