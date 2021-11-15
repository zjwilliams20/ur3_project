import numpy as np


def char_range(c1, c2):
    """Generates the characters from `c1` to `c2`, inclusive."""
    if ord(c2) > ord(c1):
        iters = range(ord(c1), ord(c2)+1, 1)
    else:
        iters = range(ord(c1), ord(c2)-1, -1)

    return [chr(c) for c in iters]


def num_range(n1, n2):
    """Generates the numbers `n1` to `n2`, inclusive."""
    n1 = int(n1)
    n2 = int(n2)

    if n2 > n1:
        iters = range(n1, n2+1, 1)
    else:
        iters = range(n1, n2-1, -1)
    
    return [str(i) for i in iters]


def gen_moves(start, end):
    """Generates the sequence of moves from start to end. This assumes the moves are valid."""

    assert start != end, 'Invalid move from %s to %s.' % (start, end)

    rows = char_range(start[0], end[0])
    cols = num_range(start[1], end[1])

    # Account for column or row moves.
    if len(rows) == 1:
        rows = rows * len(cols)
    if len(cols) == 1:
        cols = cols * len(rows)

    # Merge rows and columns into one list.
    moves = []
    for (n, b) in zip(rows, cols):
        moves.append(n+b)

    return moves

# PosMat = np.array([     [[A8,loc,BRL],[B8,loc,BKL],[C8,loc,BBL],[D8,loc,BQ ],[E8,loc,BX ],[F8,loc,BBR],[G8,loc,BKR],[H8,loc,BRR]],
#                         [[A7,loc,BP1],[B7,loc,BP2],[C7,loc,BP3],[D7,loc,BP4],[E7,loc,BP5],[F7,loc,BP6],[G7,loc,BP7],[H7,loc,BP8]],
#                         [[A6,loc,0  ],[B6,loc,0  ],[C6,loc,0  ],[D6,loc,0  ],[E6,loc,0  ],[F6,loc,0  ],[G6,loc,0  ],[H6,loc,0  ]],
#                         [[A5,loc,0  ],[B5,loc,0  ],[C5,loc,0  ],[D5,loc,0  ],[E5,loc,0  ],[F5,loc,0  ],[G5,loc,0  ],[H5,loc,0  ]],
#                         [[A4,loc,0  ],[B4,loc,0  ],[C4,loc,0  ],[D4,loc,0  ],[E4,loc,0  ],[F4,loc,0  ],[G4,loc,0  ],[H4,loc,0  ]],
#                         [[A3,loc,0  ],[B3,loc,0  ],[C3,loc,0  ],[D3,loc,0  ],[E3,loc,0  ],[F3,loc,0  ],[G3,loc,0  ],[H3,loc,0  ]],
#                         [[A2,loc,WP1],[B2,loc,WP2],[C2,loc,WP3],[D2,loc,WP4],[E2,loc,WP5],[F2,loc,WP6],[G2,loc,WP7],[H2,loc,WP8]],
#                         [[A1,loc,WRL],[B1,loc,WKL],[C1,loc,WBL],[D1,loc,WQ ],[E1,loc,WX ],[F1,loc,WBR],[G1,loc,WKR],[H1,loc,WRR]]       ])


BRL = 209 
BRR = 210
BKL = 211
BKR = 212
BBL = 213
BBR = 214
BP1 = 201
BP2 = 202
BP3 = 203
BP4 = 204
BP5 = 205
BP6 = 206
BP7 = 207
BP8 = 208
BQ = 216
BX = 215

#orentation of sitting across from the robot 
top_left = 102
top_right = 100
bottom_left = 103
bottom_right = 101

WRL = 309
WRR = 310
WKL = 311
WKR = 312
WBL = 313
WBR = 314
WP1 = 301
WP2 = 302
WP3 = 303
WP4 = 304
WP5 = 305
WP6 = 306
WP7 = 307
WP8 = 308
WQ = 315
WX = 316

s = 50
h = 80
y = 0 
x_o = 146
y_o = 33
Pos_Dict = {
    "A1": [x_o+s*7, y_o,     h],
    "A2": [x_o+s*6, y_o,     h],
    "A3": [x_o+s*5, y_o,     h],
    "A4": [x_o+s*4, y_o,     h],
    "A5": [x_o+s*3, y_o,     h],
    "A6": [x_o+s*2, y_o,     h],
    "A7": [x_o+s*1, y_o,     h],
    "A8": [x_o+s*0, y_o,     h],
    
    "B1": [x_o+s*7, y_o+s*1, h],
    "B2": [x_o+s*6, y_o+s*1, h],
    "B3": [x_o+s*5, y_o+s*1, h],
    "B4": [x_o+s*4, y_o+s*1, h],
    "B5": [x_o+s*3, y_o+s*1, h],
    "B6": [x_o+s*2, y_o+s*1, h],
    "B7": [x_o+s*1, y_o+s*1, h],
    "B8": [x_o+s*0, y_o+s*1, h],

    "C1": [x_o+s*7, y_o+s*2, h],
    "C2": [x_o+s*6, y_o+s*2, h],
    "C3": [x_o+s*5, y_o+s*2, h],
    "C4": [x_o+s*4, y_o+s*2, h],
    "C5": [x_o+s*3, y_o+s*2, h],
    "C6": [x_o+s*2, y_o+s*2, h],
    "C7": [x_o+s*1, y_o+s*2, h],
    "C8": [x_o+s*0, y_o+s*2, h],

    "D1": [x_o+s*7, y_o+s*3, h],
    "D2": [x_o+s*6, y_o+s*3, h],
    "D3": [x_o+s*5, y_o+s*3, h],
    "D4": [x_o+s*4, y_o+s*3, h],
    "D5": [x_o+s*3, y_o+s*3, h],
    "D6": [x_o+s*2, y_o+s*3, h],
    "D7": [x_o+s*1, y_o+s*3, h],
    "D8": [x_o+s*0, y_o+s*3, h],

    "E1": [x_o+s*7, y_o+s*4, h],
    "E2": [x_o+s*6, y_o+s*4, h],
    "E3": [x_o+s*5, y_o+s*4, h],
    "E4": [x_o+s*4, y_o+s*4, h],
    "E5": [x_o+s*3, y_o+s*4, h],
    "E6": [x_o+s*2, y_o+s*4, h],
    "E7": [x_o+s*1, y_o+s*4, h],
    "E8": [x_o+s*0, y_o+s*4, h],

    "F1": [x_o+s*7, y_o+s*5, h],
    "F2": [x_o+s*6, y_o+s*5, h],
    "F3": [x_o+s*5, y_o+s*5, h],
    "F4": [x_o+s*4, y_o+s*5, h],
    "F5": [x_o+s*3, y_o+s*5, h],
    "F6": [x_o+s*2, y_o+s*5, h],
    "F7": [x_o+s*1, y_o+s*5, h],
    "F8": [x_o+s*0, y_o+s*5, h],

    "G1": [x_o+s*7, y_o+s*6, h],
    "G2": [x_o+s*6, y_o+s*6, h],
    "G3": [x_o+s*5, y_o+s*6, h],
    "G4": [x_o+s*4, y_o+s*6, h],
    "G5": [x_o+s*3, y_o+s*6, h],
    "G6": [x_o+s*2, y_o+s*6, h],
    "G7": [x_o+s*1, y_o+s*6, h],
    "G8": [x_o+s*0, y_o+s*6, h],

    "H1": [x_o+s*7, y_o+s*7, h],
    "H2": [x_o+s*6, y_o+s*7, h],
    "H3": [x_o+s*5, y_o+s*7, h],
    "H4": [x_o+s*4, y_o+s*7, h],
    "H5": [x_o+s*3, y_o+s*7, h],
    "H6": [x_o+s*2, y_o+s*7, h],
    "H7": [x_o+s*1, y_o+s*7, h],
    "H8": [x_o+s*0, y_o+s*7, h]
}


def obstructed_move(start,end,piece): 

    if piece[1] == "K":     #knight jumps all
        return "move"

    else: 
        x123 = gen_moves(start,end)
        #check path to target destination
        #check each element ie chess square in the moves list below using CV 
        moves = moves[1:]

        for i in range(len(moves))
            #check square with CV cmd here

            # if square is empty:
            #     pass
            # elif  i == (len(moves) - 1) and square is not empty
            #    return "take piece"

            # else:
            #     return "obstructed"

    return "move"



def legal_move(start,end,piece): 

    #Positive is right
    deltaH = ord(dest[0]) - ord(pos[0])
    #Positive is up
    deltaV = ord(dest[1]) - ord(pos[1])


    if piece[1] == "R":     #rook
        if deltaH != 0 and deltaV != 0:
            raise AssertionError("illegal Move: %s from %s --> %s" % (piece, pos, dest))
        else:
            return "move"

    elif piece[1] =="K":    #knight     
        if abs(deltaH) == 2 and abs(deltaV) == 1:
            return "move" 
        elif abs(deltaH) == 1 and abs(deltaV) == 2:
            return "move"
        else:
            raise AssertionError("illegal Move: %s from %s --> %s" % (piece, pos, dest))

    elif piece[1] == "B":   #bishop
        if abs(deltaH) == abs(deltaV):
            return "move"
        else:
            raise AssertionError("illegal Move: %s from %s --> %s" % (piece, pos, dest))

    elif piece[1] == "Q":   #queen
        if abs(deltaH) == abs(deltaV):
            return "move"
        elif deltaH != 0 and deltaV == 0:
            return "move"
        elif deltaH == 0 and deltaV != 0:
            return "move"
        else: 
            raise AssertionError("illegal Move: %s from %s --> %s" % (piece, pos, dest))

    elif piece[1] == "X":   #king
        if abs(deltaH) > 1 or abs(deltaV) > 1:
            raise AssertionError("illegal Move: %s from %s --> %s" % (piece, pos, dest))
        else:
            return "move"

    elif piece[1] == "P":   #pawn
        if  deltaH == 0 and abs(deltaV) == 1:  
              if piece[:2] == "BP" and deltaV < 0:
                  return "move"
              elif  piece[:2] == "WP" and deltaV > 0:
                  return "move"
              else:
                  raise AssertionError("illegal Move: %s from %s --> %s" % (piece, pos, dest))
        
        elif obstructed_move(start,end,piece) == "take piece" and abs(deltaV) = 1:
              if piece[:2] == "BP" and deltaV == -1 and abs(deltaH) == 1:
                  return "move"
              elif  piece[:2] == "WP" and deltaV == 1 and abs(deltaH) == 1:
                  return "move"
              else:
                  raise AssertionError("illegal Move: %s from %s --> %s" % (piece, pos, dest))

        else:
          raise AssertionError("illegal Move: %s from %s --> %s" % (piece, pos, dest))


    else:
        return "blank"

    return 

