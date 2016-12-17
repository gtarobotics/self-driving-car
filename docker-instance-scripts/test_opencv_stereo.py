import numpy as np
import cv2
import argparse
import imutils


procWidth = 640   # processing width (x resolution) of frame
procHeight = 480

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

# disparity range is tuned for 'aloe' image pair
window_size = 3
min_disp = 16
num_disp = 112-min_disp
stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
    numDisparities = num_disp,
    blockSize = 16,
    P1 = 8*3*window_size**2,
    P2 = 32*3*window_size**2,
    disp12MaxDiff = 1,
    uniquenessRatio = 10,
    speckleWindowSize = 100,
    speckleRange = 32
)
cv2.namedWindow("left")
cv2.namedWindow("right")

cv2.moveWindow("left",0,procHeight)
cv2.moveWindow("right",procWidth,procHeight)

imgLCnt = 0
imgRCnt = 0
skipLeft=False
skipRight=False

x1=0
y1=100
x2=1920
y2=1080-320
#read first frame, it will be skipped
ret, imgL = camera.read()
'''
cv2.imshow('left',imgL)
key = cv2.waitKey(1)
while(key & 0xFF != ord('q')):
    key = cv2.waitKey(1)
'''
cv2.rectangle(imgL,(x1,y1), (x2,y2),(0,0,255))   
imgL = imgL[y1:y2, x1:x2]
imgL = imutils.resize(imgL, width=procWidth)

ret1, imgR = camera1.read()
cv2.rectangle(imgR,(x1,y1), (x2,y2),(0,0,255))   
imgR = imgR[y1:y2, x1:x2]
imgR = imutils.resize(imgR, width=procWidth)

while(True):

    if skip != 0:
        if(skip>0):
            skip=skip-1
            skipLeft=True
        else:
            skip=skip+1
            skipRight=True
        #continue

    # Capture frame-by-frame
    if(skipLeft):
        skipLeft=False
    else:
        ret, imgL = camera.read()
        cv2.rectangle(imgL,(x1,y1), (x2,y2),(0,0,255))
        imgL = imgL[y1:y2, x1:x2]   
        imgL = imutils.resize(imgL, width=procWidth)
        imgLCnt = imgLCnt + 1

    if(skipRight):
        skipRight=False
    else:
        ret1, imgR = camera1.read()
        cv2.rectangle(imgR,(x1,y1), (x2,y2),(0,0,255))
        imgR = imgR[y1:y2, x1:x2]   
        imgR = imutils.resize(imgR, width=procWidth)
        imgRCnt = imgRCnt + 1


    #imgL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    #imgR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)
    '''
    stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
    disparity = stereo.compute(imgL,imgR)
    
    
    stereo = cv2.StereoBM(1, 16, 15)
    disparity = stereo.compute(imgL, imgR)
    '''
    # compute disparity
    #disparity = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
    #disparity = (disparity-min_disp)/num_disp

    #print('computing disparity...')
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0

    ''''
    #print('generating 3d point cloud...',)
    h, w = imgL.shape[:2]
    f = 0.8*w                          # guess for focal length
    Q = np.float32([[1, 0, 0, -0.5*w],
                    [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                    [0, 0, 0,     -f], # so that y-axis looks up
                    [0, 0, 1,      0]])
    points = cv2.reprojectImageTo3D(disp, Q)
    colors = cv2.cvtColor(imgL, cv2.COLOR_BGR2RGB)
    mask = disp > disp.min()
    out_points = points[mask]
    out_colors = colors[mask]
    #out_fn = 'out.ply'
    #write_ply('out.ply', out_points, out_colors)
    #print('%s saved' % 'out.ply')
    ''' 
    cv2.imshow('disparity', (disp-min_disp)/num_disp)

    # Display the resulting imgL
    cv2.imshow('left',imgL)
    cv2.imshow('right',imgR)

    #print("Lcount %d Rcount %d Delta %d " % imgLCnt,imgLCnt,imgLCnt-imgRCnt)
    #print("Lcount %d Rcount %d Diff %d " % imgLCnt,imgRCnt,imgLCnt-imgRCnt)
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