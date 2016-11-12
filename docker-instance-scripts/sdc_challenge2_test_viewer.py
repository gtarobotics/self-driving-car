#
# Prediction code for the ai-world-team-c2
#
import argparse
import numpy as np
import pygame
import tensorflow as tf
import scipy
import scipy.misc
import cv2
import skimage.transform as sktf
import os

# Ros libraries
import roslib
import rospy
# Ros Messages
from sensor_msgs.msg import CompressedImage

#import arcgisscripting
import signal
from common import anorm2, draw_str
from time import clock

import utils


def ctrlc(sig, frame):
    raise KeyboardInterrupt("CTRL-C!")


OUTPUT_WIDTH = 160
OUTPUT_HIGH = 120
OUTPUT_CHAN = 3

image_pub = rospy.Publisher("/center_camera/image_color/compressed",
            CompressedImage)
rospy.init_node('image_feature', anonymous=True)


# from keras.models import model_from_json

pygame.init()
#size = (320 * 3, 240)
size = (640, 480)
pygame.display.set_caption("Udacity SDC challenge 2: results CSV/model viewer")
screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)

#imgleft = pygame.surface.Surface((320, 240), 0, 24).convert()
#imgcenter = pygame.surface.Surface((320, 240), 0, 24).convert()
imgcenter = pygame.surface.Surface((640, 480), 0, 24).convert()
#imgright = pygame.surface.Surface((320, 240), 0, 24).convert()
#pimg = np.zeros(shape=(320, 240, 3))
pimg = np.zeros(shape=(640, 480, 1))

# ***** get perspective transform for images *****

rsrc = \
 [[43.45456230828867, 118.00743250075844],
  [104.5055617352614, 69.46865203761757],
  [114.86050156739812, 60.83953551083698],
  [129.74572757609468, 50.48459567870026],
  [132.98164627363735, 46.38576532847949],
  [301.0336906326895, 98.16046448916306],
  [238.25686790036065, 62.56535881619311],
  [227.2547443287154, 56.30924933427718],
  [209.13359962247614, 46.817221154818526],
  [203.9561297064078, 43.5813024572758]]
rdst = \
[[10.822125594094452, 1.42189132706374],
  [21.177065426231174, 1.5297552836484982],
  [25.275895776451954, 1.42189132706374],
  [36.062291434927694, 1.6376192402332563],
  [40.376849698318004, 1.42189132706374],
  [11.900765159942026, -2.1376192402332563],
  [22.25570499207874, -2.1376192402332563],
  [26.785991168638553, -2.029755283648498],
  [37.033067044190524, -2.029755283648498],
  [41.67121717733509, -2.029755283648498]]

tform3_img = sktf.ProjectiveTransform()
tform3_img.estimate(np.array(rdst), np.array(rsrc))

def perspective_tform(x, y):
  p1, p2 = tform3_img((x,y))[0]
  return p2, p1

def  img_set_at(img, x_y, color):
  img.set_at((int(x_y[0]),int(x_y[1])), color)

# ***** functions to draw lines *****
def draw_pt(img, x, y, color, shift_from_mid, sz=1):
  col, row = perspective_tform(x, y)
  row = int(row) + shift_from_mid
  col = int(col+img.get_height()*2)/3
  if row >= 0 and row < img.get_width()-sz and\
     col >= 0 and col < img.get_height()-sz:
    img_set_at(img,(row-sz,col-sz), color)
    img_set_at(img,(row-sz,col), color)
    img_set_at(img,(row-sz,col+sz), color)
    img_set_at(img,(row,col-sz), color)
    img_set_at(img,(row,col), color)
    img_set_at(img,(row,col+sz), color)
    img_set_at(img,(row+sz,col-sz), color)
    img_set_at(img,(row+sz,col), color)
    img_set_at(img,(row+sz,col+sz), color)

def draw_path(img, path_x, path_y, color, shift_from_mid):
  for x, y in zip(path_x, path_y):
    draw_pt(img, x, y, color, shift_from_mid)

# ***** functions to draw predicted path *****

def calc_curvature(v_ego, angle_steers, angle_offset=0):
  deg_to_rad = np.pi/180.
  slip_fator = 0.0014 # slip factor obtained from real data
  steer_ratio = 15.3  # from http://www.edmunds.com/acura/ilx/2016/road-test-specs/
  wheel_base = 2.67   # from http://www.edmunds.com/acura/ilx/2016/sedan/features-specs/

  angle_steers_rad = (angle_steers - angle_offset) * deg_to_rad
  curvature = angle_steers_rad/(steer_ratio * wheel_base * (1. + slip_fator * v_ego**2))
  return curvature

def calc_lookahead_offset(v_ego, angle_steers, d_lookahead, angle_offset=0):
  #*** this function returns the lateral offset given the steering angle, speed and the lookahead distance
  curvature = calc_curvature(v_ego, angle_steers, angle_offset)

  # clip is to avoid arcsin NaNs due to too sharp turns
  y_actual = d_lookahead * np.tan(np.arcsin(np.clip(d_lookahead * curvature, -0.999, 0.999))/2.)
  return y_actual, curvature

def draw_path_on(img, speed_ms, angle_steers, angle_predicted, color1=(0,0,255), color2=(0,255,0), shift_from_mid=0):
  path_x = np.arange(0., 50.1, 0.5)
  path_y, _ = calc_lookahead_offset(speed_ms, angle_steers, path_x)

  ppath_x = np.arange(0., 50.1, 0.5)
  ppath_y, _ = calc_lookahead_offset(speed_ms, angle_predicted, ppath_x)

  draw_path(img, path_x, path_y, color1, shift_from_mid)
  draw_path(img, ppath_x, ppath_y, color2, shift_from_mid)


def pygame_to_cvimage(surface):
    """Convert a pygame surface into a cv image"""
    cv_image = cv2.CreateImageHeader(surface.get_size(), cv.IPL_DEPTH_8U, 3)
    image_string = surface_to_string(surface)
    cv2.SetData(cv_image, image_string)
    return cv_image

def surface_to_string(surface):
    """Convert a pygame surface into string"""
    return pygame.image.tostring(surface, 'RGB')

def get_pycv_image(frame):
        
        # Find the frame's dimensions in (w, h) format.
        frameSize = frame.shape[1::-1]
        
        # Convert the frame to RGB, which Pygame requires.
        #if utils.isGray(frame):
        #    conversionType = cv2.COLOR_GRAY2RGB
        #else:
        conversionType = cv2.COLOR_BGR2RGB
        rgbFrame = cv2.cvtColor(frame, conversionType)
        
        # Convert the frame to Pygame's Surface type.
        pygameFrame = pygame.image.frombuffer(
            rgbFrame.tostring(), frameSize, 'RGB')
        
        # Resize the window to match the frame.
        #displaySurface = pygame.display.set_mode(frameSize)
        
        # Blit and display the frame.
        #displaySurface.blit(pygameFrame, (0, 0))
        #pygame.display.flip()

        return pygameFrame

track_len = 10
detect_interval = 5
tracks = []
frame_idx = 0
prev_gray=0

lk_params = dict( winSize  = (15, 15),
                  maxLevel = 2,
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

feature_params = dict( maxCorners = 500,
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )


def lk_track(frame):
    global track_len,detect_interval,tracks,frame_idx,prev_gray,lk_params,feature_params
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    vis = frame.copy()

    if len(tracks) > 0:
        img0, img1 = prev_gray, frame_gray
        p0 = np.float32([tr[-1] for tr in tracks]).reshape(-1, 1, 2)
        p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
        p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
        d = abs(p0-p0r).reshape(-1, 2).max(-1)
        good = d < 1
        new_tracks = []
        for tr, (x, y), good_flag in zip(tracks, p1.reshape(-1, 2), good):
            if not good_flag:
                continue
            tr.append((x, y))
            if len(tr) > track_len:
                del tr[0]
            new_tracks.append(tr)
            cv2.circle(vis, (x, y), 3, (0, 0, 255), -1)
        tracks = new_tracks
        cv2.polylines(vis, [np.int32(tr) for tr in tracks], False, (0, 255, 0))
        draw_str(vis, (250, 300), 'track count: %d' % len(tracks))

    if frame_idx % detect_interval == 0:
        mask = np.zeros_like(frame_gray)
        mask[:] = 255
        for x, y in [np.int32(tr[-1]) for tr in tracks]:
            cv2.circle(mask, (x, y), 5, 0, -1)
        p = cv2.goodFeaturesToTrack(frame_gray, mask = mask, **feature_params)
        if p is not None:
            for x, y in np.float32(p).reshape(-1, 2):
                tracks.append([(x, y)])


    frame_idx += 1
    prev_gray = frame_gray
    #cv2.imshow('lk_track', vis)
    return vis


imfiles=[]
steers=[]


nvidiaMode = True

#import model change this to point to your model
if nvidiaMode:
    import model_mslavescu as model
else:
    import model_aiwagan as model

#'''
sess = tf.InteractiveSession()
saver = tf.train.Saver()
if nvidiaMode:
    saver.restore(sess, "./save_mslavescu/model.ckpt")
else:
    saver.restore(sess, "./save_aiwagan/model.ckpt")


trainingSet="0" #0 is the test set, 100 is training set and 1000 is Nvidia AutoPilot Dataset
traininSetFolder = "/sharefolder/sdc/sdc-data/Train/"+trainingSet+"/"
def loadDatasetInfo(traininSetFolder,steeringAngleFileName):
    imfiles=[]
    steers=[]
    with open(traininSetFolder+ steeringAngleFileName) as f:
        for line in f:
            if line.startswith("frame_id") :
                continue
            fields = line.split(',')
            if len(fields) == 2:
                imfiles.append(line.split(',')[0])
                steers.append(float(line.split(',')[1]))# * 180/ scipy.pi) #rad to deg
            else:
                imfiles.append(line.split(' ')[0])
                steers.append(float(line.split(' ')[1]))# * 180/ scipy.pi) #rad to deg
    return (imfiles,steers)

(imfiles,steers) = loadDatasetInfo(traininSetFolder,"submission_7_mslavescu.csv") 

img = cv2.imread('steering_wheel_image.jpg',0)
rows,cols = img.shape

smoothed_angle = 0

index = -1
pred = False

try:

    print("frame_id,steering_angle")

    for imageName in imfiles:
        index=index+1
        imagePath =traininSetFolder+"center/" +imageName + ".jpg"
        if(not os.path.exists(imagePath)):
            imagePath =traininSetFolder+"center/" +imageName + ".png"
        #print(imagePath)

        if(os.path.exists(imagePath)):
            origSteerAngle = steers[index] #  * scipy.pi / 180
            if(pred==True):
                #img_for_pred = scipy.misc.imresize(scipy.misc.imread(imagePath), [120, 160])/255.0
                full_image = scipy.misc.imread(imagePath, mode="RGB")
                if nvidiaMode:
                    image = scipy.misc.imresize(full_image[-150:], [66, 200]) / 255.0
                    steerAngle = model.y.eval(feed_dict={model.x: [image], model.keep_prob: 1.0})[0][0]
                    #steerAngle = -steerAngle #we need this for Nvidia model
                    #steerAngle = steerAngle * scipy.pi / 180 #deg to rad 
                else:
                    image = scipy.misc.imresize(full_image[-150:], [66, 200]) / 255.0
                    steerAngle = model.y.eval(feed_dict={model.x: [image], model.keep_prob: 1.0})[0][0]
            else:
                steerAngle = origSteerAngle
            
            origSteerAngleDeg = origSteerAngle * 180/ scipy.pi #rad to deg
            steerAngleDeg = steerAngle * 180/ scipy.pi #rad to deg

            angles = "%2.2f  %2.2f %2.2f" % (steerAngleDeg,origSteerAngleDeg,origSteerAngleDeg-steerAngleDeg)
            #print(imageName+" - steerAngle: "+str(steerAngle)+" - angles: "+angles)

            timestampAngle = "%s,%.15f" % (imageName,steerAngle)
            print(timestampAngle)

            if False: #steerAngle!=0:
                smoothed_angle += 0.2 * pow(abs((steerAngleDeg - smoothed_angle)), 2.0 / 3.0) * (steerAngleDeg - smoothed_angle) / abs(steerAngleDeg - smoothed_angle)
                #smoothed_angle=-steerAngleDeg
                M = cv2.getRotationMatrix2D((cols/2,rows/2),smoothed_angle,1)
                dst = cv2.warpAffine(img,M,(cols,rows))
                cv2.imshow("steering wheel", dst)


            cv_image = cv2.imread(imagePath)
            #cv_image = cv2.resize(cv_image,(320, 240), interpolation = cv2.INTER_CUBIC)
            
            cv_image = lk_track(cv_image)
            cv2.imshow('lk_track', cv_image)

            xshift=0
            yshift=-20
            xtop = 245
            pts = np.array([[xshift+0,yshift+460],[xshift+20,yshift+480],[xshift+xtop,yshift+300],[xshift+xtop-20,yshift+280],[xshift+0,yshift+460]])
            cv2.polylines(cv_image,[pts],True,(0,255,255))
            
            xshift=0
            yshift=-20
            pts = np.array([[xshift+640,yshift+460],[xshift+640-20,yshift+480],[xshift+640-xtop,yshift+300],[xshift+640-(xtop-20),yshift+280],[xshift+640,yshift+460]])
            cv2.polylines(cv_image,[pts],True,(0,255,255))

            cv_image = cv2.resize(cv_image,(640, 480), interpolation = cv2.INTER_CUBIC)

            draw_str(cv_image, (10, 470), imageName)

            draw_str(cv_image, (260, 320), angles)
            
            imgcenter = get_pycv_image(cv_image)

            amplif = 1

            draw_path_on(imgcenter, 0,amplif*origSteerAngleDeg,amplif*steerAngleDeg,(0,255,0), (0,0,255),160) #recoreded where available
            
            screen.blit(imgcenter, (0,0))

            ch = 0xFF & cv2.waitKey(1)
            if ch == 27:
                signal.signal(signal.SIGINT, ctrlc)
                break

            #uncomment pygame.image.save line to save the proceessed images and later create a video like this:
            #https://www.youtube.com/watch?v=1vQJ_ssuEBg
            #Then run the following in a gtrarobotics/udacity-sdc:vnc docker image container
            #~/sharefolder/sdc/sdc-data/Train/0/capture# ls -1 | sort -n | awk 'BEGIN{i=0} {printf("ln -s %s img%04d.tga\n",$1,i);i++}' | sh
            #~/sharefolder/sdc/sdc-data/Train/0/capture# ffmpeg -r 25 -i img%04d.tga -c:v libx264 -vf fps=25 -pix_fmt yuv420p out.mp4
            #pygame.image.save(screen, traininSetFolder+'capture/'+imageName)
            pygame.display.flip()

except SystemExit:
    pygame.quit()