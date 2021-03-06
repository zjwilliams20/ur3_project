#!/usr/bin/python
# scratch.py

import numpy as np
import matplotlib.pyplot as plt

import cv2
import cv2.aruco as aruco

from geometry_msgs.msg import Point
from std_msgs.msg import Int32MultiArray

import os

test_dir = os.path.dirname(os.path.realpath(__file__)) + '/'
# IMG_PATH = test_dir + 'white-pieces.png'
# IMG_PATH = test_dir + 'black-pieces.png'
# IMG_PATH = test_dir + 'singlemarkersdetection.jpg'
IMG_PATH = test_dir + 'board-test.png'
# IMG_PATH = test_dir + 'board-test2.png'

ARUCO_DICT = {
	"DICT_4X4_50": aruco.DICT_4X4_50,
	"DICT_4X4_100": aruco.DICT_4X4_100,
	"DICT_4X4_250": aruco.DICT_4X4_250,
	"DICT_4X4_1000": aruco.DICT_4X4_1000,
	"DICT_5X5_50": aruco.DICT_5X5_50,
	"DICT_5X5_100": aruco.DICT_5X5_100,
	"DICT_5X5_250": aruco.DICT_5X5_250,
	"DICT_5X5_1000": aruco.DICT_5X5_1000,
	"DICT_6X6_50": aruco.DICT_6X6_50,
	"DICT_6X6_100": aruco.DICT_6X6_100,
	"DICT_6X6_250": aruco.DICT_6X6_250,
	"DICT_6X6_1000": aruco.DICT_6X6_1000,
	"DICT_7X7_50": aruco.DICT_7X7_50,
	"DICT_7X7_100": aruco.DICT_7X7_100,
	"DICT_7X7_250": aruco.DICT_7X7_250,
	"DICT_7X7_1000": aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": aruco.DICT_ARUCO_ORIGINAL
	# "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	# "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	# "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	# "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

CHESS_DICT = ARUCO_DICT["DICT_ARUCO_ORIGINAL"]


def find_aruco_dict(img):
    """Determine the most likely aruco dictionary for a cv2 image."""

    aruco_params = aruco.DetectorParameters_create()
    for key in ARUCO_DICT:
        corners, ids, rejected = aruco.detectMarkers(
            img, 
            aruco.Dictionary_get(ARUCO_DICT[key]), 
            parameters=aruco_params
            )
        if len(corners):
            print('%s\t-->\t%d detections' % (key, len(corners)))
            # print(ids)


def aruco_detect(image, do_annotation=True):
    """Detect all aruco markers in the current image."""

    aruco_params = aruco.DetectorParameters_create()
    aruco_params.minMarkerDistanceRate = 0.015

    aruco_dict = aruco.Dictionary_get(CHESS_DICT)
    corners, ids, rejected = aruco.detectMarkers(image, aruco_dict, parameters=aruco_params)

    # Remove detections with strange ID's, sometimes 3 horizontal squares gets picked up as 1023
    valid_mask = ids <= 1000
    if ids is not None:
        ids = ids[valid_mask]
        corners = [corner for i, corner in enumerate(corners) if valid_mask[i]]

    # Draw the frames of the detected markers
    if do_annotation:
        _ = aruco.drawDetectedMarkers(image, corners, ids)
        # _ = aruco.drawDetectedMarkers(image, rejected, np.arange(len(rejected)))
    
    return ids, np.array(corners)
    


def corners2points(corners):
    '''Convert array of corners into a 2D list of points'''
    points = []
    for ibox, box in enumerate(corners):
        points.append([Point(x=corner[0], y=corner[1]) for corner in box[0]])
    return points


if __name__ == '__main__':
    
    img = cv2.imread(IMG_PATH)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # find_aruco_dict(img)

    ids, corners = aruco_detect(img)
    # print(ids)

    # points = []
    # for ibox, box in enumerate(corners):
    #     points.append([Point(x=corner[0], y=corner[1]) for corner in box[0]])

    cv2.imshow('image', img)
    cv2.waitKey(0)


    
