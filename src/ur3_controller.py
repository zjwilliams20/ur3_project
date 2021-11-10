# ur3_utils.py
# utilities for controlling the UR3 arm


import time
from functools import reduce

import numpy as np
from numpy import cos, sin, arccos, arcsin, arctan2, hypot
from scipy.linalg import expm

import rospy
from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input

from header import *


class Ur3Controller(object):
    """UR3 arm control related functionality and associated data."""

    n_joints = N_JOINTS
    IO_0 = False

    def __init__(self, loop):
        self.thetas = np.zeros(self.n_joints)
        self.curr_dest = None
        self.suck_set = False
        self.pos_set = False
        self.loop = loop

        self.cmd_pub = rospy.Publisher('ur3/command', command, queue_size=10)
        self.pos_sub = rospy.Subscriber('ur3/position', position, self.position_cb)
        self.grip_sub = rospy.Subscriber('ur3/gripper_input', gripper_input, self.gripper_cb)

    def position_cb(self, msg):
        """Whenever ur3/position publishes info, update member data."""

        self.thetas = np.array(msg.position)
        self.pos_set = True

    def gripper_cb(self, msg):
        """When /ur3/gripper_input publishes something, update gripper I/O globals."""

        self.suck_set = msg.DIGIN

    def move_theta(self, theta_dest):
        """Move arm to joint angles specified by destination."""

        error = False
        spin_count = 0
        at_goal = False
        self.curr_dest = theta_dest

        driver_msg = command(destination=theta_dest, v=VEL, a=ACC, io_0=self.suck_set)
        self.cmd_pub.publish(driver_msg)
        self.loop.sleep()

        while not at_goal:

            if all(abs(self.thetas - theta_dest) < DIST_TOL):
                at_goal = True
                rospy.loginfo("Goal is reached!")
            self.loop.sleep()

            if spin_count > SPIN_RATE*5:

                self.cmd_pub.publish(driver_msg)
                rospy.loginfo("Just published again driver_msg")
                spin_count = 0
            spin_count += 1

        return error

    def move_pos(self, pos_dest, yaw=135):
        """Move arm to position specified using forward and inverse kinematics."""

        theta_dest = self._invk(pos_dest, yaw)
        self.move_theta(theta_dest)

    def sucker(self, suck_set=False):
        """Turn suction on or off"""

        self.suck_set = suck_set
        
        # NOTE: we use move_theta, since io_0 and destination are combined in the command message.
        return self.move_theta(self.curr_dest)

    def _fk(self, theta):
        """Function that calculates encoder numbers for each motor."""
        
        expos = [expm(S[i] * theta[i]) for i in range(self.n_joints)]
        expo = reduce(np.matmul, expos)

        T = np.matmul(expo, M)
        # print(str(np.round(T, 2)) + "\n")

        return theta + np.array([np.pi, 0, 0, -0.5*np.pi, 0, 0])

    def _invk(self, pWgrip, yaw_WgripDegree):
        """Function that calculates an elbow up Inverse Kinematic solution for the UR3."""
        
        yaw_WgripRads = np.radians(yaw_WgripDegree)

        # ----- 1. Position of gripper wrt world
        p_baseW = pWgrip - np.array([[-150, 150, 10]]).T * 1e-3

        # ----- 2. Position of center point wrt world
        p_cenW = p_baseW + np.array([
            [-L9*cos(yaw_WgripRads)],
            [-L9*sin(yaw_WgripRads)],
            [0]
        ])

        # ----- 3. Angle between Joint 1 wrt x_w
        theta_cen = arctan2(p_cenW[1], p_cenW[0])[0]
        theta_c2 = arcsin(LC / hypot(*p_cenW[:2]))[0]
        theta_1 = theta_cen - theta_c2

        # ----- 4. Joint 6 angle
        theta_6 = np.pi/2 + theta_1 - yaw_WgripRads

        # ----- 5. Position of center point
        z_3endW_offset = L8 + L10
        p_3endW = p_cenW + np.array([
            [LC*sin(theta_1) - L7*cos(theta_1)],
            [-(LC*cos(theta_1) + L7*sin(theta_1))],
            [z_3endW_offset]
        ])

        # ----- 6. Compute Joint angles 2-4
        z_3hypo = p_3endW[2] - L1
        x_3hypo = p_3endW[0] / cos(theta_1)
        h_3end = hypot(x_3hypo, z_3hypo)[0]
        
        beta = arctan2(z_3hypo, x_3hypo)[0]
        alpha = loc(L3, h_3end, L5)
        gamma = loc(L5, h_3end, L3)
        phi = loc(L3, L5, h_3end)

        theta_2 = -(alpha + beta)
        theta_3 = np.pi - phi
        theta_4 = -(gamma - beta)
        assert theta_4 - (theta_3 - theta_2) <= EPS, '-(gamma - beta) must == theta_3 - theta_2.'

        theta = [theta_1, theta_2, theta_3, theta_4, np.radians(-90), theta_6]
        print([np.degrees(t) for t in theta])
        return self._fk(theta)

    def demo(self):
        """Demonstrate move functionality of ur3 arm controller."""

        self.move_theta(home)
        time.sleep(1)
        self.move_theta(board)
        self.sucker(SUCKER_ON)
        time.sleep(1)
        self.move_theta(home)
        time.sleep(1)

        pos = np.array([220, 220, 220]) * 1e-3
        self.move_pos(pos)
        time.sleep(1)
        self.move_theta(home)
        time.sleep(1)

        