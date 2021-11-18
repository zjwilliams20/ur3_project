# chess.py

import rospy

import numpy as np

from image_manager import ImageManager
from ur3_controller import Ur3Controller
from ur3_project.msg import Aruco, Box

# Convert to numpy array in meters.
from chess_func import POS_DICT, KNIGHTS
POS_DICT = dict(map(lambda m: (m[0], 1e-3*np.array(m[1]).T), POS_DICT.items()))

import chess_func as cf
from header import *


class ChessGame(object):
    """Chess subsystem to represent chess board and keep up with game."""

    def __init__(self):
        # Dictionary of pieces associated to their chess positions, e.g. 207: A1.
        self.board = None
        self.aruco_sub = rospy.Subscriber('aruco_id', Aruco, self.aruco_cb)

        # Other necessary sub-systems for game execution.
        self.ur3_ctrl = Ur3Controller()
        self.img_mgr = ImageManager()

    def aruco_cb(self, msg):
        """Update the chess dictionary whenever aruco_id gets published to."""

        # Dictionary of pieces detected from Aruco and their x, y coordinates.
        pieces = {}
        for m in msg.boxes:
            pieces[m.id] = np.array([[m.corners[0].x, m.corners[0].y]]).T

        # Associate detected pieces to hardcoded board positions.
        pos_ids = POS_DICT.keys()
        pos_arr = np.array(POS_DICT.values())
        pos_arr = pos_arr[:,:2] # remove z-dimension
        pos_arr = pos_arr.T.reshape(1, 2, -1) # extend in 3D

        piece_ids = pieces.keys()
        piece_arr = np.array(pieces.values())

        # n_pieces x n_positions
        dists = np.linalg.norm(pos_arr - piece_arr, axis=1)
        mins = np.argmin(dists, axis=1)
        assert len(set(mins)) == len(mins), 'Duplicate piece assignment detected.'

        self.board = {pos_ids[m]: piece_ids[i] for i, m in enumerate(mins)}

        # Alternative LSA association that I started but shouldn't be needed.
        # row_inds, col_inds = linear_sum_assignment(dists)
        # print(row_inds, col_inds)
        # print([piece_ids[r] for r in row_inds])
        # print([pos_ids[c] for c in col_inds])

    def validate_move(self, start, end):
        """Validation checks on chess move."""

        piece = self.board[start]
        if self.obstructed_move(start, end, piece):
            print('Obstructed.')
            return False        
        if not cf.legal_move(start, end, piece):
            print('Illegal')
            return False
        return True

    def obstructed_move(self, start, end, piece_id):
        """Determine whether there are pieces in the way moving from start to end."""

        print(piece_id, KNIGHTS)
        if piece_id in KNIGHTS: # knight jumps all
            return False

        # Generate moves from start to end.        
        moves = cf.gen_moves(start, end)
        # print(self.board)
        # print(moves)
        for move in moves[:-1]:
            # Check each element ie chess square in the moves list below using CV 
            print(move)
            if move in self.board:
                return True
        return False

    def move_piece(self, start, end):
        """
        1. user input 
        2. validate_move
        3. move_block
        """

        # T/F of whether the piece is on the board.
        take_piece = end in self.board
        off_board = [0, 0, 0, 0]

        if take_piece:
            self.ur3_mgr.move_block(end,off_board) # fling it
            self.ur3_mgr.move_block(start,end)
        else: 
            self.ur3_mgr.move_block(start,end)

    def play(self):
        """Play the chess game with the ur3_controller and the image manager."""

        self.ur3_ctrl.move_theta(home)

        while self.board is None:
            rospy.loginfo('Waiting for camera...')
            rospy.sleep(1)

        # start, end = cf.user_input()
        start = 'B1'
        end = 'C3'
        print('Start: ' + start + '\tEnd: ' + end)

        if self.validate_move(start, end):
            print('Valid move.')
        else:
            print('Invalid move.')

        self.move_piece(start, end)

        # # self.move_pos(self.pieces[ID_GRAB])
        # start_pos = self.pieces[ID_GRAB] # A7
        # end_pos = np.array([POS_DICT['E6'][:3]]).T

        # self.move_block(start_pos, end_pos)
        # print(start_pos, end_pos)

