#!/usr/bin/env python
#!/usr/bin/python
"""
sdc_rosbag_viewer.py: version 0.1 by Marius Slavescu

"""

import rospy

import pygame, sys
from pygame.locals import *
from sensor_msgs.msg import CompressedImage
import numpy as np
import cStringIO

#Initiates the display
pygame.init()
windowSurfaceObj= pygame.display.set_mode((320*3,240))
pygame.display.set_caption('SDC Viewer')
yellow = pygame.Color(245,210,0)
windowSurfaceObj.fill(pygame.Color(0,0,0))
pygame.display.update()

imgleft = pygame.surface.Surface((320,240),0,24).convert()
imgcenter = pygame.surface.Surface((320,240),0,24).convert()
imgright = pygame.surface.Surface((320,240),0,24).convert()

def compressedImageCB(ros_data):
        fstr = cStringIO.StringIO(ros_data.data)
        if ros_data._connection_header['topic'] == '/left_camera/image_color/compressed':
                imgleft = pygame.transform.scale(pygame.image.load(fstr), (320, 240))		
                windowSurfaceObj.blit(imgleft, (0,0))
        elif ros_data._connection_header['topic'] == '/center_camera/image_color/compressed':
                imgcenter = pygame.transform.scale(pygame.image.load(fstr), (320, 240))
                windowSurfaceObj.blit(imgcenter, (320,0))
        elif ros_data._connection_header['topic'] == '/right_camera/image_color/compressed':
                imgright = pygame.transform.scale(pygame.image.load(fstr), (320, 240))		
                windowSurfaceObj.blit(imgright, (640,0))

        #pygame.display.flip() #
	pygame.display.update()


def listener():

	rospy.init_node('odom_graph', anonymous=True)

        rospy.Subscriber("/left_camera/image_color/compressed", CompressedImage, compressedImageCB)
        rospy.Subscriber("/center_camera/image_color/compressed", CompressedImage, compressedImageCB)
        rospy.Subscriber("/right_camera/image_color/compressed", CompressedImage, compressedImageCB)

	rospy.spin()

if __name__=="__main__":
	
	listener()
		
