import numpy as np
import cv2
import argparse
import imutils


procWidth = 640   # processing width (x resolution) of frame

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video", help="path to the video file")

args = vars(ap.parse_args())

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

# if the video argument is None, then we are reading from webcam
if args.get("video", None) is None:
    camera = cv2.VideoCapture(0)
    time.sleep(2)

# otherwise, we are reading from a video file
else:
    portNr = args.get("video")
    if is_number(portNr):
        camera = cv2.VideoCapture(int(portNr))
    else:
        camera = cv2.VideoCapture(portNr)

while(True):
    # Capture frame-by-frame
    ret, frame = camera.read()
    frame = imutils.resize(frame, width=procWidth)

    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()