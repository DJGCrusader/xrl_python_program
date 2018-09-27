'''
Reads once and converts voltages to total loads.
Last updated 3/24/2017 (OBSOLETE)
Has J1 and J2
'''

import numpy as np
import bridgeInterface as b_i
import time

# Geometry
angle_between_beams = 60  # degrees
beam_tilt_angle = 90 - 19.50935  # degrees
spacing_angle = 120  # degrees
plate_spacing = 0.2  # meters

t = 0.5 * np.deg2rad(angle_between_beams)  # rad
b = np.deg2rad(beam_tilt_angle)  # rad
p = 0.5 * np.deg2rad(spacing_angle)  # rad
a = plate_spacing / (1 + np.cos(p))
g = np.sqrt(1 + (np.tan(t)) ** 2 + (1 / (np.tan(b))) ** 2)

# Getting values
b_i.initialize()

print("Place object on sensor!!")
time.sleep(5)
[R1, L1, R2, L2, R3, L3] = b_i.getCalibVals()  # raw voltages from each sensor
print("Voltages with weight: ", [R1, L1, R2, L2, R3, L3])

# As a columm [N_R1 N_vL1 N_vR2 N_vL2 N_vR3 N_vL3] compression = (+)
N = np.array([[R1], [L1], [R2], [L2], [R3], [L3]])
Ja = np.array([[1/g, 1/g], [-np.tan(t)/g, np.tan(t)/g], [-1/(np.tan(b)*g), -1/(np.tan(b)*g)]])
J1 = np.concatenate((np.concatenate((Ja, np.zeros((3, 4))), axis = 1), np.concatenate((np.zeros((3, 2)), Ja, np.zeros((3, 2))), axis = 1), np.concatenate((np.zeros((3, 4)), Ja), axis = 1)), axis = 0)
F = np.dot(J1, N)  # F = [Fx1; Fy1; Fz1; Fx2; Fy2; Fz2; Fx3; Fy3; Fz3]
J2 = np.array([[1, 0, 0, 1, 0, 0, 1, 0, 0],
    [0, 1, 0, 0, -np.sin(p), np.cos(p), 0, -np.sin(p), -np.cos(p)],
    [0, 0, 1, 0, -np.cos(p), -np.sin(p), 0, np.cos(p), -np.sin(p)],
    [0, -a, 0, 0, -a, 0, 0, -a, 0],
    [a, 0, 0, -a * np.cos(p), 0, 0, -a * np.cos(p), 0, 0],
    [0, 0, 0, 0, 0, -a * np.sin(p), 0, 0, a * np.sin(p)]])
# print(Ja)
# print(F)
# print(J2)
L = np.dot(J2, F)
print("Final loads: ", L)
