# header.py
# Constants and utility functions

import numpy as np
from numpy import arccos

EPS = np.finfo(np.float32).eps

skew = lambda v: np.array([
	[0, -v[2], v[1]],
	[v[2], 0, -v[0]],
	[-v[1], v[0], 0]
])
bracket = lambda omega, v: np.vstack([
	np.hstack([skew(omega), v]), np.zeros((1,4))
	])
    
deg2rad = lambda theta: theta * np.pi/180
loc = lambda a, b, c: arccos((a**2 + b**2 - c**2) / (2*a*b))

# --- Inverse Kinematics ---
# link lengths
L1 = 152e-3
# L2 = 120e-3 # unused
L3 = 244e-3
# L4 = 93e-3 # unused
L5 = 213e-3
L6 = 83e-3
L7 = 83e-3
L8 = 82e-3
L9 = 53.5e-3
L10 = 59e-3

# distance from Joint 5 to p_3end
L_END = 27e-3

# distance from p_3end to center point along link 6
LC = L6 + L_END

SPIN_RATE = 20 # Hz
DIST_TOL = 0.0005
DELAY = 0.1
VEL = 4
ACC = 4

SUCKER_ON = True
SUCKER_OFF = False

home = np.radians([270, -45, 45, -90, -90, 135])
board = np.radians([180, -45, 45, -90, -90, 135])

N_JOINTS = 6

# --- Forward Kinematics ---
R = np.array([
    [0, -1, 0],
    [0, 0, -1],
    [1, 0, 0]
])
p = np.array([[390, 401, 215.5]]).T * 1e-3

pad = np.array([[0, 0, 0, 1]])
M = np.vstack([np.hstack([R, p]), pad])

omega = np.array([
    [0, 0, 1],
    [0, 1, 0],
    [0, 1, 0],
    [0, 1, 0],
    [1, 0, 0],
    [0, 1, 0]
])

v = np.array([
    [0.15, 0.15, 0],
    [-0.162, 0, -0.15],
    [-0.162, 0, 0.094],
    [-0.162, 0, 0.307],
    [0, 0.162, -0.26],
    [-0.162, 0, 0.39]
])

S = [bracket(omega[i], v[i,np.newaxis].T) for i in range(N_JOINTS)]


# --- Image Processing ---
BETA = 745.0 # pixels / meter
THETA = np.radians(0)

OR = -240.0
OC = -320.0

TX = -(OR + 62.0)
TY = -(OC + 179.0)
ZW = 0.0

RCW = np.array([
    [np.cos(THETA), -np.sin(THETA),     0],
    [np.sin(THETA),  np.cos(THETA),     0],
    [0            ,  0            ,     1]
])

ORIGIN = (-int(OC), -int(OR))
ORIGIN_W = (int(-(TY + OC)), int(-(TX + OR)))
RED = (0, 0, 255)
BLUE = (255, 0, 0)

# STAGE_LOC = np.radians(
#     [163.58, -77.66, 95.96, -108.9, -90.0, 75.06]
# )
STAGE_LOC = board

NO_ERR = 0

Z_OFFSET = 0.100