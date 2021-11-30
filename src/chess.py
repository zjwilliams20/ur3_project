# chess.py

import rospy

import numpy as np

from image_manager import ImageManager
from ur3_controller import Ur3Controller
from ur3_project.msg import Aruco, Box

# Convert to numpy array in meters.
from chess_func import POS_DICT, KNIGHTS, PAWNS, BLK_PAWNS, WHT_PAWNS, PAWNS, ROOKS, BISHOPS, QUEENS, KINGS

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
        if len(set(mins)) != len(mins):
            return

        self.board = {pos_ids[m]: piece_ids[i] for i, m in enumerate(mins)}
        # print(self.board)

        # Alternative LSA association that I started but shouldn't be needed.
        # row_inds, col_inds = linear_sum_assignment(dists)
        # print(row_inds, col_inds)
        # print([piece_ids[r] for r in row_inds])
        # print([pos_ids[c] for c in col_inds])

    def validate_move(self, start, end, piece_id):
        """Validation checks on chess move."""
        deltaH = abs(ord(end[0]) - ord(start[0]))
        #Positive is up
        deltaV = abs(ord(end[1]) - ord(start[1]))

        piece = self.board[start]

        if self.obstructed_move(start, end, piece) and deltaV == 1 and deltaH == 1 and piece_id in PAWNS:
            return True

        elif self.obstructed_move(start, end, piece) and deltaH == 0 and deltaV == 1 and piece_id in PAWNS:
            print('The piece you wish to move cannot jump other pieces. Please enter a new move.')
            return False  

        elif self.obstructed_move(start, end, piece):
            print('The piece you wish to move cannot jump other pieces. Please enter a new move.')
            return False  


        if not self.legal_move(start, end, piece):
            print('The piece you are trying to move cannot legally move in that path.')
            return False

        return True

    def obstructed_move(self, start, end, piece_id):
        """Determine whether there are pieces in the way moving from start to end."""
   
        #print(piece_id, KNIGHTS)
        if piece_id in KNIGHTS: # knight jumps all
            return False

        # Generate moves from start to end.        
        moves = cf.gen_moves(start, end)
        # print(self.board)
        # print(moves)
        if piece_id in PAWNS:
            for move in moves:
                # Check each element ie chess square in the moves list below using CV 
                #print("move list: " + str(move))
                if move in self.board:
                    return True
        else:
            for move in moves[:-1]:
                # Check each element ie chess square in the moves list below using CV 
                #print("move list: " + str(move))
                if move in self.board:
                    return True
        
        return False

    def legal_move(self, start, end, piece_id): 
        """Determine whether a move is permitted based on the chess rules, the piece."""

        #Positive is right
        deltaH = ord(end[0]) - ord(start[0])
        #Positive is up
        deltaV = ord(end[1]) - ord(start[1])

        if piece_id in ROOKS:     #rook
            if deltaH != 0 and deltaV != 0:
                return False
            else:
                return True

        elif piece_id in KNIGHTS:    #knight    
            if abs(deltaH) == 2 and abs(deltaV) == 1:
                return True 
            elif abs(deltaH) == 1 and abs(deltaV) == 2:
                return True
            else:
                return False

        elif piece_id in BISHOPS:   #bishop
            if abs(deltaH) == abs(deltaV):
                return True
            else:
                return False

        elif piece_id in QUEENS:   #queen
            if abs(deltaH) == abs(deltaV):
                return True
            elif deltaH != 0 and deltaV == 0:
                return True
            elif deltaH == 0 and deltaV != 0:
                return True
            else: 
                return False

        elif piece_id in KINGS:   #king
            if abs(deltaH) > 1 or abs(deltaV) > 1:
                return False
            else:
                return True

        elif piece_id in PAWNS:   #pawn
            if  deltaH == 0 and abs(deltaV) == 1 and not self.obstructed_move(start, end, piece_id):  
                  if piece_id in BLK_PAWNS and deltaV < 0:
                      return True
                  elif  piece_id in WHT_PAWNS and deltaV > 0:
                      return True
                  else:
                      return False

            elif self.obstructed_move(start, end, piece_id) and abs(deltaV) == 1 and abs(deltaH) == 1:
                  if piece_id in BLK_PAWNS and deltaV == -1 and abs(deltaH) == 1:
                      return True
                  elif  piece_id in WHT_PAWNS and deltaV == 1 and abs(deltaH) == 1:
                      return True
                  else:
                      return False
            elif start[1] == "7" and abs(deltaV) == 2: 
                return True 

            else:
                return False

        else:
            assert False, 'Unreachable.'


    def move_piece(self, start, end):
        """
        1. user input 
        2. validate_move
        3. move_block
        """

        # T/F of whether the piece is on the board.
        global counter
        take_piece = end in self.board
        off_board = np.array( [0, 0, 0, 0] ).T
        piece_id_srt = self.board[start]
        
        

        if take_piece:
            piece_id_end = self.board[end]
            self.ur3_ctrl.move_block(end,off_board,piece_id_end) # fling it
            self.ur3_ctrl.move_block(start,end,piece_id_srt)
        

        else: 
            self.ur3_ctrl.move_block(start,end,piece_id_srt)

    def user_input(self): 
        """prompt virtual player for a start and end location"""

        start = 0
        end = 0 

        while start == end:

            start = 0
            end = 0 

            while start == 0:

                input_string_srt = raw_input("Enter a start coordinate using an uppercase letter and an integer: ")
                srt = input_string_srt in self.board
            
                if len(input_string_srt) != 2: #more than 2 character input 
                    print("Please enter a start coordinate 2 characters long, an uppercase letter and an integer")

                elif ord(input_string_srt[0]) > 90:  #entered upper case letters or to high of numbers
                    print("Please use upper case letters in your start coordinate")

                elif ord(input_string_srt[0]) > 72 or int(input_string_srt[1]) > 8 or int(input_string_srt[1]) < 1: #not on board
                    print("That start coordinate is not on the board")
                
                elif not srt:
                    print("That is not a valid start position")
        
                else: 
                    start = input_string_srt
                    print("Start Coordinate noted")
                    


            while end == 0: 

                input_string_end = raw_input("Enter an end coordinate using an uppercase letter and an integer: ")
                ends = input_string_end in self.board

                if len(input_string_end) != 2: #more than 2 character input 
                    print("Please enter an end coordinate 2 characters long, an uppercase letter and an integer")

                elif ord(input_string_end[0]) > 90:  #entered upper case letters or to high of numbers
                    print("Please use upper case letters in your start coordinate")

                elif ord(input_string_end[0]) > 72 or int(input_string_end[1]) > 8 or int(input_string_srt[1]) < 1: #not on board
                    print("That end coordinate is not on the board")

                elif start == input_string_end:
                    print("You have to move.")
                
                elif ends:
                        piece_id_endi = self.board[input_string_end]
                        if piece_id_endi < 300:
                            print("Another one of your pieces already occupies that end position")
                        else:
                            end = input_string_end
                            print("End Coordinate noted")

                else: 
                    end = input_string_end
                    print("End Coordinate noted")

        start, end = input_string_srt, input_string_end

        print("RoboBoi will move the piece from the start coordinate " + input_string_srt + " to the end coordinate " + input_string_end)
        print("")
        return start, end 


    def play(self):
        """Play the chess game with the ur3_controller and the image manager."""

        self.ur3_ctrl.move_theta(home)
        # right = np.radians([270, 25, -25, -90, 0, 100])
        # left = np.radians([90, 25, -25, -90, 0, 100])

        # self.ur3_ctrl.move_theta(right)
        # self.ur3_ctrl.move_theta(left)

        while self.board is None:
            rospy.loginfo('Waiting for camera...')
            rospy.sleep(1)
        
        # start = 'B1'
        # end = 'C3'
        global counter
        counter = 0 
        gameon = 0

        while gameon < 20:

            start, end = self.user_input()
            piece = self.board[start]
            while not self.validate_move(start, end, piece):
                start, end = self.user_input()
                
            self.move_piece(start,end)
            self.ur3_ctrl.move_theta(home)
            gameon = gameon + 1 

        print("Test game complete")
        exit()
        

