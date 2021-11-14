import numpy as np


# PosMat = np.array([     [[A8,loc,BRL],[B8,loc,BKL],[C8,loc,BBL],[D8,loc,BQ ],[E8,loc,BX ],[F8,loc,BBR],[G8,loc,BKR],[H8,loc,BRR]],
#                         [[A7,loc,BP1],[B7,loc,BP2],[C7,loc,BP3],[D7,loc,BP4],[E7,loc,BP5],[F7,loc,BP6],[G7,loc,BP7],[H7,loc,BP8]],
#                         [[A6,loc,0  ],[B6,loc,0  ],[C6,loc,0  ],[D6,loc,0  ],[E6,loc,0  ],[F6,loc,0  ],[G6,loc,0  ],[H6,loc,0  ]],
#                         [[A5,loc,0  ],[B5,loc,0  ],[C5,loc,0  ],[D5,loc,0  ],[E5,loc,0  ],[F5,loc,0  ],[G5,loc,0  ],[H5,loc,0  ]],
#                         [[A4,loc,0  ],[B4,loc,0  ],[C4,loc,0  ],[D4,loc,0  ],[E4,loc,0  ],[F4,loc,0  ],[G4,loc,0  ],[H4,loc,0  ]],
#                         [[A3,loc,0  ],[B3,loc,0  ],[C3,loc,0  ],[D3,loc,0  ],[E3,loc,0  ],[F3,loc,0  ],[G3,loc,0  ],[H3,loc,0  ]],
#                         [[A2,loc,WP1],[B2,loc,WP2],[C2,loc,WP3],[D2,loc,WP4],[E2,loc,WP5],[F2,loc,WP6],[G2,loc,WP7],[H2,loc,WP8]],
#                         [[A1,loc,WRL],[B1,loc,WKL],[C1,loc,WBL],[D1,loc,WQ ],[E1,loc,WX ],[F1,loc,WBR],[G1,loc,WKR],[H1,loc,WRR]]       ])


#ROBO CANT REACH FULL RANGE - PUSH CLOSER AND CHANGE x_o & y_o
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
    "A1": [x_o+s*7, y_o,     h, y, 0], 
    "A2": [x_o+s*6, y_o,     h, y, 0],
    "A3": [x_o+s*5, y_o,     h, y, 0],
    "A4": [x_o+s*4, y_o,     h, y, 0], 
    "A5": [x_o+s*3, y_o,     h, y, 0],
    "A6": [x_o+s*2, y_o,     h, y, 0],
    "A7": [x_o+s*1, y_o,     h, y, 0],
    "A8": [x_o+s*0, y_o,     h, y, 0], 
    
    "B1": [x_o+s*7, y_o+s*1, h, y, 0],
    "B2": [x_o+s*6, y_o+s*1, h, y, 0],
    "B3": [x_o+s*5, y_o+s*1, h, y, 0],
    "B4": [x_o+s*4, y_o+s*1, h, y, 0],
    "B5": [x_o+s*3, y_o+s*1, h, y, 0],
    "B6": [x_o+s*2, y_o+s*1, h, y, 0],
    "B7": [x_o+s*1, y_o+s*1, h, y, 0],
    "B8": [x_o+s*0, y_o+s*1, h, y, 0],

    "C1": [x_o+s*7, y_o+s*2, h, y, 0],
    "C2": [x_o+s*6, y_o+s*2, h, y, 0],
    "C3": [x_o+s*5, y_o+s*2, h, y, 0],
    "C4": [x_o+s*4, y_o+s*2, h, y, 0],
    "C5": [x_o+s*3, y_o+s*2, h, y, 0],
    "C6": [x_o+s*2, y_o+s*2, h, y, 0],
    "C7": [x_o+s*1, y_o+s*2, h, y, 0],
    "C8": [x_o+s*0, y_o+s*2, h, y, 0],

    "D1": [x_o+s*7, y_o+s*3, h, y, 0],
    "D2": [x_o+s*6, y_o+s*3, h, y, 0],
    "D3": [x_o+s*5, y_o+s*3, h, y, 0],
    "D4": [x_o+s*4, y_o+s*3, h, y, 0],
    "D5": [x_o+s*3, y_o+s*3, h, y, 0],
    "D6": [x_o+s*2, y_o+s*3, h, y, 0],
    "D7": [x_o+s*1, y_o+s*3, h, y, 0],
    "D8": [x_o+s*0, y_o+s*3, h, y, 0],

    "E1": [x_o+s*7, y_o+s*4, h, y, 0],
    "E2": [x_o+s*6, y_o+s*4, h, y, 0],
    "E3": [x_o+s*5, y_o+s*4, h, y, 0],
    "E4": [x_o+s*4, y_o+s*4, h, y, 0],
    "E5": [x_o+s*3, y_o+s*4, h, y, 0],
    "E6": [x_o+s*2, y_o+s*4, h, y, 0],
    "E7": [x_o+s*1, y_o+s*4, h, y, 0],
    "E8": [x_o+s*0, y_o+s*4, h, y, 0],

    "F1": [x_o+s*7, y_o+s*5, h, y, 0],
    "F2": [x_o+s*6, y_o+s*5, h, y, 0],
    "F3": [x_o+s*5, y_o+s*5, h, y, 0],
    "F4": [x_o+s*4, y_o+s*5, h, y, 0],
    "F5": [x_o+s*3, y_o+s*5, h, y, 0],
    "F6": [x_o+s*2, y_o+s*5, h, y, 0],
    "F7": [x_o+s*1, y_o+s*5, h, y, 0],
    "F8": [x_o+s*0, y_o+s*5, h, y, 0],

    "G1": [x_o+s*7, y_o+s*6, h, y, 0],
    "G2": [x_o+s*6, y_o+s*6, h, y, 0],
    "G3": [x_o+s*5, y_o+s*6, h, y, 0],
    "G4": [x_o+s*4, y_o+s*6, h, y, 0],
    "G5": [x_o+s*3, y_o+s*6, h, y, 0],
    "G6": [x_o+s*2, y_o+s*6, h, y, 0],
    "G7": [x_o+s*1, y_o+s*6, h, y, 0],
    "G8": [x_o+s*0, y_o+s*6, h, y, 0],

    "H1": [x_o+s*7, y_o+s*7, h, y, 0],
    "H2": [x_o+s*6, y_o+s*7, h, y, 0],
    "H3": [x_o+s*5, y_o+s*7, h, y, 0],
    "H4": [x_o+s*4, y_o+s*7, h, y, 0],
    "H5": [x_o+s*3, y_o+s*7, h, y, 0],
    "H6": [x_o+s*2, y_o+s*7, h, y, 0],
    "H7": [x_o+s*1, y_o+s*7, h, y, 0],
    "H8": [x_o+s*0, y_o+s*7, h, y, 0],    }


def obstructed_move(piece,pos,dest): 

    deltaH = abs(ord(dest[0]) - ord(pos[0]))
    deltaV = abs(ord(dest[1]) - ord(pos[1]))

    if piece[1] == "K":     #knight jumps all
        return "move"

    elif deltaH == deltaV:      #diagonal move
        return

    elif deltaV == 0:    #horizontal move 
        while pos != dest: 
            #pos = #change position by one square
            return 
        return

    elif deltaH == 0:    #vertical move
        return

    else: 
        return "something broken :/" 

    return "did not successfully check"

    
         


def legal_move(piece,pos,dest): 

    #Positive is right
    deltaH = ord(dest[0]) - ord(pos[0])
    #Positive is up
    deltaV = ord(dest[1]) - ord(pos[1])


    if piece[1] == "R":     #rook
        if deltaH != 0 and deltaV != 0:
            raise AssertionError("illegal Move: %s from %s --> %s" % (piece, pos, dest))

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

    elif piece[1] == "P":   #pawn
        if  deltaH == 0 and abs(deltaV) == 1:  
              if piece[:2] == "BP" and deltaV < 0:
                  return "move"
              elif  piece[:2] == "WP" and deltaV > 0:
                  return "move"
              else:
                  raise AssertionError("illegal Move: %s from %s --> %s" % (piece, pos, dest))
        
        # elif call adjacent obstruction == yes and one movement in diagonal
        #   return "move"
        
        else:
          raise AssertionError("illegal Move: %s from %s --> %s" % (piece, pos, dest))


    else:
        return ""

    return 

