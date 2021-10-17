#!/usr/bin/env python
# run.py

import numpy as np
import matplotlib.pyplot as plt

import rospy
from cv_bridge import CvBridge
import cv2

from header import *


class ImageManager():
    '''Manager of cv_camera_node publisher and subscriber, and image processing
    '''

    def __init__(self):

        # Image figure
        self.fig, self.ax = plt.subplots(1)
        # plt.show()

        # Link between opencv2 and ROS
        self.bridge = CvBridge()

        # Output of camera image
        self.image = None

        # Initialize camera publisher
        self.pub = rospy.Publisher('cv_camera/image_raw', Image, queue_size=1)

        # Initialize camera subscriber
        rospy.Subscriber('/cv_camera_node/image_raw', Image, self.camera_cb)
        rospy.loginfo('[run] Constructing topic objects...')


    def camera_cb(self, msg):
        '''Camera callback that updates camera globals
        '''

        rospy.loginfo('[camera_cb] converting msg to cv2')
        self.image = self.bridge.imgmsg_to_cv2(msg)
        self.view()
        rospy.sleep(1)


    def view(self):
        '''View the current image
        '''

        if self.image is None:
            return
        
        rospy.loginfo('[view] viewing image...')
        plt.sca(self.ax)
        plt.cla()
        cv2.imshow('ImageManager', self.image)
        cv2.waitKey()


if __name__ == '__main__':

    rospy.loginfo('[run] Starting main...')

    # Initialize ROS node
    rospy.init_node('main')

    # Initialize publisher for ur3/command with buffer size of 10
    pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

    # Spin up image manager
    img_mgr = ImageManager()

    rospy.spin()
    rospy.loginfo('[run] Done.')

