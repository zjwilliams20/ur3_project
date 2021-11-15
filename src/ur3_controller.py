# ur3_controller.py
# utilities for controlling the UR3 arm


import time
from warnings import warn
from functools import reduce

import numpy as np
from numpy import cos, sin, arccos, arcsin, arctan2, hypot
from scipy.linalg import expm
from scipy.optimize import linear_sum_assignment

import rospy
from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input
from ur3_project.msg import Aruco, Box

from header import *
import chess_func as cf

# Convert to numpy array in meters.
from chess_func import Pos_Dict as POS_DICT
POS_DICT = dict(map(lambda m: (m[0], 1e-3*np.array(m[1]).T), POS_DICT.items()))

SUCTION_ON = True
SUCTION_OFF = False


class Ur3Controller(object):
    """UR3 arm control related functionality and associated data."""

    n_joints = N_JOINTS
    IO_0 = False

    def __init__(self, loop):
        self.thetas = np.zeros(self.n_joints)
        self.curr_dest = None
        self.suck_set = False
        self.suck_det = False
        self.pos_set = False
        self.pieces = None
        self.loop = loop

        self.cmd_pub = rospy.Publisher('ur3/command', command, queue_size=10)
        self.pos_sub = rospy.Subscriber('ur3/position', position, self.position_cb)
        self.grip_sub = rospy.Subscriber('ur3/gripper_input', gripper_input, self.gripper_cb)
        self.aruco_sub = rospy.Subscriber('aruco_id', Aruco, self.aruco_cb)

    def position_cb(self, msg):
        """Whenever ur3/position publishes info, update member data."""

        self.thetas = np.array(msg.position)
        self.pos_set = True

    def gripper_cb(self, msg):
        """When /ur3/gripper_input publishes something, update gripper I/O data."""

        self.suck_det = msg.DIGIN

    def aruco_cb(self, msg):
        """Update the chess dictionary whenever aruco_id gets published to."""

        self.pieces = {}
        for m in msg.boxes:
            self.pieces[m.id] = np.array([[m.corners[0].x, m.corners[0].y]]).T

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

    def move_pos(self, pos_dest, yaw=0):
        """Move arm to position specified using forward and inverse kinematics."""

        theta_dest = self._invk(pos_dest, yaw)
        self.move_theta(theta_dest)

    def move_block(self, start, end):
        """Move block from starting to end position."""

        offset = np.array([
            [0, 0, Z_OFFSET]
        ]).T
        start_stage = start + offset
        end_stage = end + offset

        self.move_pos(start_stage)
        self.move_pos(start)
        self.sucker(SUCTION_ON)
        self.move_pos(start_stage)

        if not self.suck_det:
            self.sucker(SUCTION_OFF)
            self.move_pos(start_stage)
            warn("Couldn't find block...")
            return NO_ERR

        self.move_pos(end)
        time.sleep(1)
        self.sucker(SUCTION_OFF)
        time.sleep(2)
        self.move_pos(end_stage)

        return NO_ERR

    def sucker(self, suck_set=False):
        """Turn suction on or off."""

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

        return self._fk([theta_1, theta_2, theta_3, theta_4, np.radians(-90), theta_6])

    def validate_move(self,start,end,piece):
        """
        1. Is the the move obstructed?
        2. Can the pieces do that?
        3. Is there a pieces there?
        """
        ov = cf.obstructed_move(start,end,piece)
        lmv = cf.legal_move(start,end,piece)

        if lmv != "move":
            return "Not a legal move"
        elif ov == "obstructed":
            return "The move is obstructed"
        return None
        

    def usr_input(self): 
        """prompt virtual player for a start and end location"""
        start = 0
        end = 0 

        while start == end:

            while start == 0:

                input_string_srt = raw_input("Enter a start coordinate using an uppercase letter and an integer: ")
  
                if len(input_string_srt) != 2: #more than 2 character input 
                    print("Please enter a start coordinate 2 characters long, an uppercase letter and an integer")

                elif ord(input_string_srt[0]) > 90:  #entered upper case letters or to high of numbers
                    print("Please use upper case letters in your start coordinate")

                elif ord(input_string_srt[0]) > 72 or int(input_string_srt[1]) > 8: #not on board
                    print("That start coordinate is not on the board")

                else: 
                    start = input_string_srt
                    print("Start Coordinate noted")


            while end == 0: 

                input_string_end = raw_input("Enter an end coordinate using an uppercase letter and an integer: ")

                if len(input_string_end) != 2: #more than 2 character input 
                    print("Please enter an end coordinate 2 characters long, an uppercase letter and an integer")

                elif ord(input_string_end[0]) > 90:  #entered upper case letters or to high of numbers
                    print("Please use upper case letters in your start coordinate")

                elif ord(input_string_end[0]) > 72 or int(input_string_end[1]) > 8: #not on board
                    print("That end coordinate is not on the board")

                else: 
                    end = input_string_end
                    print("End Coordinate noted")

        start, end = input_string_srt, input_string_end

        #call CV matrix to return piece

        print("RoboBoi will move the piece from the start coordinate " + input_string_srt + " to the end coordinate " + input_string_end)
        return start, end 


    def move_piece(self):
        """
        1. user input 
        2. validate_move
        3. move_block
        """
        start, end = self.user_input()
        #self.validate_move() needs to raise an error

        o = cf.obstructed_move(start,end,piece)   #need to source from other file
        o = 0 
        off_board = [0,0,0,0]

        if o == "take piece":
            self.move_block(self,end,off_board)     #fling it
            self.move_block(self,start,end)
        else: 
            self.move_block(self,start,end)

        pass

    def demo(self):
        """Demonstrate move functionality of ur3 arm controller."""

        ID_GRAB = 209
        # pos = np.array([[0.1, 0., 0.1]]).T # (220, 64)
        # pos = np.array([[0.2, 0.0, 0.1]]).T # (220, 159)
        self.move_theta(home)

        while self.pieces is None:
            rospy.loginfo('Waiting...')
            rospy.sleep(0.5)

        # Associate detected pieces to hardcoded board positions.
        pos_ids = POS_DICT.keys()
        pos_arr = np.array(POS_DICT.values())
        pos_arr = pos_arr[:,:2] # remove z-dimension
        pos_arr = pos_arr.T.reshape(1, 2, -1) # extend in 3D

        piece_ids = self.pieces.keys()
        piece_arr = np.array(self.pieces.values())

        # n_pieces x n_positions
        dists = np.linalg.norm(pos_arr - piece_arr, axis=1)
        # mins = np.argmin(dists, axis=1)
        # print(len(set(mins)), len(mins))
        # assert len(set(mins)) == len(mins), 'Duplicate piece assignment detected.'

        row_inds, col_inds = linear_sum_assignment(dists)
        print(row_inds, col_inds)
        print([piece_ids[r] for r in row_inds])
        print([pos_ids[c] for c in col_inds])

        # assignments = list(POS_DICT)[mins]
        # print(assignments)

        # while self.pieces is None or ID_GRAB not in self.pieces:
        #     rospy.loginfo('Waiting...')
        #     time.sleep(0.5)
        # print(self.pieces)

        # # self.move_pos(self.pieces[ID_GRAB])
        # start_pos = self.pieces[ID_GRAB] # A7
        # end_pos = np.array([POS_DICT['E6'][:3]]).T

        # self.move_block(start_pos, end_pos)
        # print(start_pos, end_pos)


        