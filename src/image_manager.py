# image_manager.py

import rospy
from cv_bridge import CvBridge

import numpy as np
import matplotlib.pyplot as plt
import cv2
import cv2.aruco as aruco

from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from ur3_project.msg import Aruco, Box

from header import *


Z_PIECE = 0

WHITE_PIECES = range(301, 317)
BLACK_PIECES = range(201, 217)
MISC_MARKERS = range(100, 104)


class ImageManager(object):
    """Manager of cv_camera_node publisher and subscriber, and image processing."""
    
    CHESS_DICT = aruco.DICT_ARUCO_ORIGINAL
    CALLBACK_DELAY = 0.1

    def __init__(self):

        # Image figure.
        self.fig, self.ax = plt.subplots(1)
        self.bridge = CvBridge()

        # Image to include in the processing pipeline.
        self.proc_image = None
        # Image to display for sanity checks.
        self.raw_image = None

        # Initialize necessary publishers and subscribers.
        self.img_pub = rospy.Publisher('cv_camera/image_raw', Image, queue_size=1)
        self.img_sub = rospy.Subscriber('/cv_camera_node/image_raw', Image, self.camera_cb)
        self.aruco_pub = rospy.Publisher('aruco_id', Aruco, queue_size=1)

    def camera_cb(self, msg):
        """Camera callback that captures puts an image into OpenCV and searches for
           aruco markers."""

        self.raw_image = self.bridge.imgmsg_to_cv2(msg)

        # Flip the image to be consistent with Lab 5.
        self.raw_image = cv2.flip(self.raw_image, -1)
        self.raw_image = enlarge(self.raw_image, 200)

        self.proc_image = self.raw_image.copy()
        self.proc_image = cv2.cvtColor(self.proc_image, cv2.COLOR_BGR2GRAY)
        _, self.proc_image = cv2.threshold(self.proc_image, 127, 255, cv2.THRESH_BINARY)

        ids, corners = self.aruco_detect()
        boxes = corners2boxes(ids, corners)
        self.aruco_pub.publish(boxes=boxes)

        self.view()
        rospy.sleep(self.CALLBACK_DELAY)

    def aruco_detect(self, do_annotation=True):
        """Detect all aruco markers in the current image."""

        corners = None
        
        aruco_params = aruco.DetectorParameters_create()
        aruco_dict = aruco.Dictionary_get(self.CHESS_DICT)
        corners, ids, rejected = aruco.detectMarkers(self.proc_image, aruco_dict, parameters=aruco_params)

        # Remove detections with strange ID's, sometimes 3 horizontal squares gets picked up as 1023
        valid_mask = ids <= 1000
        if ids is not None:
            ids = ids[valid_mask]
            corners = [corner for i, corner in enumerate(corners) if valid_mask[i]]

        # Draw the frames of the detected markers
        if do_annotation:
            # _ = aruco.drawDetectedMarkers(self.raw_image, corners, ids)
            _ = aruco.drawDetectedMarkers(self.raw_image, rejected, np.arange(len(rejected)))
        
        return ids, np.array(corners)

    def view(self):
        """View the current image with annotations."""

        if self.proc_image is None:
            return
        
        # Annotate image origin
        draw_plus(self.proc_image, ORIGIN, BLUE)
        draw_plus(self.proc_image, ORIGIN_W, RED)

        cv2.imshow('Processed Image', self.proc_image)
        cv2.imshow('Raw Image', self.raw_image)
        cv2.waitKey(1)

        # c1 = np.array([220, 64])
        # c2 = np.array([220, 159])
        # c1 = np.array([534, 106.25])
        # c2 = np.array([258, 105])

        # beta, theta = calibrate(c1, c2)
        # print('beta: %.3g, theta= %.3g' % (beta, theta))


def IMG2W(r, c):
    '''Map a position in the camera frame to the world frame
    '''

    pc = np.array([
        [(r + OR + TX) / BETA],
        [(c + OC + TY) / BETA],
        [ZW]
    ])

    return np.matmul(RCW, pc)


def corners2boxes(ids, corners):
    """Convert array of corners into a box messages."""

    boxes = []
    for ibox, box_corners in enumerate(corners):
        # Compute box centroid.
        centroid = np.array([np.mean(box_corners[0,:,0]), np.mean(box_corners[0,:,1])])
        # Convert to world coordinates.
        # print(ids[ibox], centroid[0], centroid[1])
        centroid = IMG2W(int(centroid[1]), int(centroid[0]))
        boxes.append(Box(ids[ibox], [Point(x=centroid[0], y=centroid[1], z=centroid[2])]))
    return boxes


def draw_plus(image, point, color=RED):
    """Draw a plus at the specified point."""

    WIDTH = 10

    left = (point[0]-WIDTH, point[1])
    right = (point[0]+WIDTH, point[1])
    upper = (point[0], point[1]+WIDTH)
    lower = (point[0], point[1]-WIDTH)

    cv2.line(image, left, right, color, thickness=1)
    cv2.line(image, lower, upper, color, thickness=1)


def enlarge(image, percent):
    """Grow an image along x and y at the specified percentage."""
    
    width = int(image.shape[1] * percent / 100)
    height = int(image.shape[0] * percent / 100)
    dim = (width, height)
    
    # Enlarge image to help out Aruco.
    # interp = cv2.INTER_AREA
    # interp = cv2.INTER_NEAREST
    # interp = cv2.INTER_LINEAR
    interp = cv2.INTER_CUBIC
    return cv2.resize(image, dim, interpolation=interp)


def calibrate(centroid_1, centroid_2):
    '''Compute beta as the ratio between pixels to distance
    '''

    # left & right in camera's frame, which is y in world frame
    centroid_L = centroid_1 if centroid_1[1] < centroid_2[1] else centroid_2
    centroid_R = centroid_2 if centroid_1 is centroid_L else centroid_1

    beta = np.linalg.norm(centroid_1 - centroid_2) / 370.5e-3
    theta = np.degrees(np.arctan2(centroid_R[0] - centroid_L[0], centroid_R[1] - centroid_L[1]))

    return beta, theta