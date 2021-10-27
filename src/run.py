#!/usr/bin/env python
# run.py

import numpy as np
import matplotlib.pyplot as plt
import cv2
import cv2.aruco as aruco

import rospy
from cv_bridge import CvBridge

from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input

from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import String
from ur3_project.msg import ids


CHESS_DICT = aruco.DICT_ARUCO_ORIGINAL
CALLBACK_DELAY = 0.1
Z_PIECE = 0

WHITE_PIECES = range(301, 317)
BLACK_PIECES = range(201, 217)
MISC_MARKERS = range(100, 104)


class ImageManager():
    '''Manager of cv_camera_node publisher and subscriber, and image processing
    '''

    def __init__(self):

        # Image figure
        self.fig, self.ax = plt.subplots(1)
        self.bridge = CvBridge()

        # Output of camera image
        self.image = None

        rospy.loginfo('[ImageManager.__init__] Constructing topic objects...')

        # Initialize necessary publishers and subscribers
        self.img_pub = rospy.Publisher('cv_camera/image_raw', Image, queue_size=1)
        self.img_sub = rospy.Subscriber('/cv_camera_node/image_raw', Image, self.camera_cb)
        self.aruco_pub = rospy.Publisher('aruco_id', ids, queue_size=1)


    def camera_cb(self, msg):
        '''Camera callback that captures puts an image into OpenCV and searches for
           aruco markers.
        '''

        self.image = self.bridge.imgmsg_to_cv2(msg)

        rospy.loginfo('[camera_cb] detecting aruco markers')
        ids, corners = self.aruco_detect()
        points = corners2points(corners)
        self.aruco_pub.publish(ids=ids, boxes=points)

        self.view()
        rospy.sleep(CALLBACK_DELAY)


    def aruco_detect(self, do_annotation=True):
        '''Detect all aruco markers in the current image
        '''

        aruco_params = aruco.DetectorParameters_create()
        aruco_dict = aruco.Dictionary_get(CHESS_DICT)
        corners, ids, _ = aruco.detectMarkers(self.image, aruco_dict, parameters=aruco_params)

        # Remove detections with strange ID's, sometimes 3 horizontal squares gets picked up as 1023
        valid_mask = ids <= 1000
        ids = ids[valid_mask]
        corners = [corner for i, corner in enumerate(corners) if valid_mask[i]]

        # Draw the frames of the detected markers
        if do_annotation:
            _ = aruco.drawDetectedMarkers(self.image, corners, ids)
        
        return ids, corners


    def view(self):
        '''View the current image with annotations
        '''

        if self.image is None:
            return
        
        rospy.loginfo('[view] viewing image...')
        # plt.sca(self.ax)
        # plt.cla()
        cv2.imshow('ImageManager', self.image)
        cv2.waitKey(1)


def corners2points(corners):
    '''Convert array of corners into a 2D list of points'''
    points = []
    for ibox, box in enumerate(corners):
        points.append([Point(x=corner[0], y=corner[1]) for corner in box[0]])
    return points


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

