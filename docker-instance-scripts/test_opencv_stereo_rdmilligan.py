import numpy as np
import cv2
import argparse
import imutils


procWidth = 640   # processing width (x resolution) of frame

# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-l", "--left_video", help="path to the left video file")
ap.add_argument("-r", "--right_video", help="path to the right video file")
ap.add_argument("-s", "--skip", help="skip left if pozitive or right if negative")

args = vars(ap.parse_args())

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

# if the video argument is None, then we are reading from webcam
if args.get("left_video", None) is None:
    camera = cv2.VideoCapture(0)
    time.sleep(2)

# otherwise, we are reading from a video file
else:
    portNr = args.get("left_video")
    if is_number(portNr):
        camera = cv2.VideoCapture(int(portNr))
    else:
        camera = cv2.VideoCapture(portNr)
    portNr = args.get("right_video")
    if is_number(portNr):
        camera = cv2.VideoCapture(int(portNr))
    else:
        camera1 = cv2.VideoCapture(portNr)

skip = int(args.get("skip"))

skipLeft=False
skipRight=False
skipLeftCount=0
skipRightCount=0

# disparity settings
window_size = 5
min_disp = 32
num_disp = 112-min_disp
stereo = cv2.StereoSGBM_create(
    minDisparity = min_disp,
    numDisparities = num_disp,
    SADWindowSize = window_size,
    uniquenessRatio = 10,
    speckleWindowSize = 100,
    speckleRange = 32,
    disp12MaxDiff = 1,
    P1 = 8*3*window_size**2,
    P2 = 32*3*window_size**2,
    fullDP = False
)
 
# morphology settings
kernel = np.ones((12,12),np.uint8)
 
counter = 450

cv2.namedWindow("left",0,480)
cv2.namedWindow("right",procWidth,480)

while(True):
    # Capture frame-by-frame
    if(skipLeft):
        skipLeft=False
    else:
        ret, imgL = camera.read()
    if(skipRight):
        skipRight=False
    else:
        ret1, imgR = camera1.read()

    if skip != 0:
        if(skip>0):
            skip=skip-1
            skipLeft=True
        else:
            skip=skip+1
            skipRight=True
        continue

    imgL = imutils.resize(imgL, width=procWidth)
    imgR = imutils.resize(imgR, width=procWidth)

    # Display the resulting imgL
    cv2.imshow('left',imgL)
    cv2.imshow('right',imgR)

    #imgL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    #imgR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    '''
    stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
    disparity = stereo.compute(imgL,imgR)
    
    
    stereo = cv2.StereoBM(1, 16, 15)
    disparity = stereo.compute(imgL, imgR)
    '''

    # compute disparity
    disparity = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
    disparity = (disparity-min_disp)/num_disp

    # apply threshold
    threshold = cv2.threshold(disparity, 0.6, 1.0, cv2.THRESH_BINARY)[1]

    # apply morphological transformation
    morphology = cv2.morphologyEx(threshold, cv2.MORPH_OPEN, kernel)

    cv2.imshow('disparity',disparity)
    cv2.imshow('threshold', threshold)
    cv2.imshow('morphology', morphology)

    #'''

    key = cv2.waitKey(1)
    if key & 0xFF == ord('q'):
        break
    if key & 0xFF == ord('l'):
        skipLeft=True
    else:
        if key & 0xFF == ord('r'):
            skipRight=True
        
# When everything done, release the capture
#cap.release()
cv2.destroyAllWindows()