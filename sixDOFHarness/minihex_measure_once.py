'''
Reads once and transforms from voltages to total loads.
Last updated 5/2017 (MOST CURRENT VERSION)
Has J1, J2, and J3; NO angle adjustment
'''

import numpy as np
import bridgeInterface as b_i
import time
import winsound

# Geometry
angle_between_beams = 60  # degrees
beam_tilt_angle = 90 - 19.50935  # degrees
spacing_angle = 120  # angle between each pari of beams; degrees
radius = 0.096  # meters
Db = 0.0484  # spacing between balls; meters
ball_to_surface = 18.09  # mm

t = 0.5 * np.deg2rad(angle_between_beams)  # rad
b = np.deg2rad(beam_tilt_angle)  # rad
alpha = 0.5 * np.deg2rad(spacing_angle)  # rad
p = np.pi / 2 - alpha  # rad
a = radius
s = ball_to_surface / 1000  # meters
# g = np.sqrt(1 + (np.tan(t)) ** 2 + (1 / (np.tan(b))) ** 2)

# Alert
Freq = 2000  # Set Frequency To 2500 Hertz
Dur = 600  # Set Duration To 1000 ms == 1 second


# Getting values
b_i.initialize()
winsound.Beep(Freq, Dur)
print("Place object on sensor!!")
time.sleep(5)

[vR1, vL1, vR2, vL2, vR3, vL3] = b_i.getCalibVals()
# print("Raw voltages with weight: ", [vR1, vL1, vR2, vL2, vR3, vL3])

# Convert voltage to weight
v = [vR1, vL1, vR2, vL2, vR3, vL3]
A = [-1.0883, -1.0559, -1.1294, -1.1070, -1.0853, -1.1272]
f = [A[i] * v[i] for i in range(len(v))]
[R1, L1, R2, L2, R3, L3] = f


# As a columm [N_R1 N_vL1 N_vR2 N_vL2 N_vR3 N_vL3] compression = (+)
N = np.array([[R1], [L1], [R2], [L2], [R3], [L3]])
Ja = np.array([[np.cos(t) * np.sin(b), np.cos(t) * np.sin(b)], 
	[-np.sin(t), np.sin(t)],
	[-np.cos(t) * np.cos(b), -np.cos(t)* np.cos(b)], 
	[0.5 * Db * np.cos(t) * np.sin(b), -0.5 * Db * np.cos(t) * np.sin(b)]])
# J1 = from individual beam forces to beam pair triangle forces
J1 = np.concatenate((np.concatenate((Ja, np.zeros((4, 4))), axis = 1), np.concatenate((np.zeros((4, 2)), Ja, np.zeros((4, 2))), axis = 1), np.concatenate((np.zeros((4, 4)), Ja), axis = 1)), axis = 0)
F = np.dot(J1, N)  # F = [Fx1; Fy1; Fz1; Fx2; Fy2; Fz2; Fx3; Fy3; Fz3]
# J2 = from beam pair triangle forces to effective forces at center on ball joint plane
J2 = np.array([[1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
    [0, 1, 0, 0, 0, -np.sin(p), np.cos(p), 0, 0, -np.sin(p), -np.cos(p), 0],
    [0, 0, 1, 0, 0, -np.cos(p), -np.sin(p), 0, 0, np.cos(p), -np.sin(p), 0],
    [0, -a, 0, 0, 0, -a, 0, 0, 0, -a, 0, 0],
    [a, 0, 0, 0, -a * np.sin(p), 0, 0, np.cos(p), -a * np.sin(p), 0, 0, -np.cos(p)],
    [0, 0, 0, 1, -a * np.cos(p), 0, 0, -np.sin(p), a * np.cos(p), 0, 0, -np.sin(p)]])
L = np.dot(J2, F)
# J3 = move up from ball joint plane to plate plane
J3 =  np.array([[1, 0, 0, 1, 0, 0, ],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, -s, 0, 1, 0],
    [0, s, 0, 0, 0, 1]])
LL = np.dot(J3, L)
LL_rounded = np.around(LL, decimals=2)

# print("Voltages per beam: ", v)
# print("Forces per beam: ", f)
# print("Effective Fs at each triangle", F)
# print("old final loads:", '\n', L)
print("Final loads:", '\n', LL_rounded)
