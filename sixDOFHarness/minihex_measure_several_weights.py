'''
Used to test sensor accuracy
Reads 3 weights, 5x each, one sample OR several samples each, and transforms from voltages to total loads. Also saves data.
Last updated 4/26/2017
Has J1, J2, and J3; and angle adjustment. REMOVED angle adjustment.
To run: Rename file/change "tested" variable. Put preload weight on. Start. High long beep = change weight. Low beep = take off and put back on.
'''

import numpy as np
import bridgeInterface as b_i
import time
import winsound
import ctypes  # An included library with Python install.
ctypes.windll.user32.MessageBoxW(0, "Close Excel! Rename file!", "Before you go...", 1)

# setup
output = np.zeros((3, 5))
fs = np.zeros((3, 6))
Fx = 0
Fy = 1
Fz = 2
Mx = 3
My = 4
Mz = 5

# -----WHICH ONE ARE YOU TESTING?------
tested = My  # Fx, Fy, Fz, Mx, My, Mz?
name = 'results_-My.csv'
name2 = 'rawresults_-My.csv'
# -------------------------------------

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
Freq1 = 524
Freq2 = 783
Freq3 = 932

b_i.initialize()

# As a columm [N_R1 N_vL1 N_vR2 N_vL2 N_vR3 N_vL3] compression = (+)
Ja = np.array([[np.cos(t) * np.sin(b), np.cos(t) * np.sin(b)], 
	[-np.sin(t), np.sin(t)],
	[-np.cos(t) * np.cos(b), -np.cos(t)* np.cos(b)], 
	[0.5 * Db * np.cos(t) * np.sin(b), -0.5 * Db * np.cos(t) * np.sin(b)]])
# J1 = from individual beam forces to beam pair triangle forces
J1 = np.concatenate((np.concatenate((Ja, np.zeros((4, 4))), axis = 1), np.concatenate((np.zeros((4, 2)), Ja, np.zeros((4, 2))), axis = 1), np.concatenate((np.zeros((4, 4)), Ja), axis = 1)), axis = 0)
# J2 = from beam pair triangle forces to effective forces at center on ball joint plane
J2 = np.array([[1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
    [0, 1, 0, 0, 0, -np.sin(p), np.cos(p), 0, 0, -np.sin(p), -np.cos(p), 0],
    [0, 0, 1, 0, 0, -np.cos(p), -np.sin(p), 0, 0, np.cos(p), -np.sin(p), 0],
    [0, -a, 0, 0, 0, -a, 0, 0, 0, -a, 0, 0],
    [a, 0, 0, 0, -a * np.sin(p), 0, 0, np.cos(p), -a * np.sin(p), 0, 0, -np.cos(p)],
    [0, 0, 0, 1, -a * np.cos(p), 0, 0, -np.sin(p), a * np.cos(p), 0, 0, -np.sin(p)]])
# J3 = move up from ball joint plane to plate plane
J3 =  np.array([[1, 0, 0, 1, 0, 0, ],
    [0, 1, 0, 0, 0, 0],
    [0, 0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, -s, 0, 1, 0],
    [0, s, 0, 0, 0, 1]])

m = 0
n = 0

for n in range(0, 3):
	print("Change weight!!")
	winsound.Beep(Freq3, 1000)  # change weight
	time.sleep(7)
	for m in range(0, 5):
		# Getting values
		winsound.Beep(Freq1, 600)  # measuring weight 5x
		time.sleep(3)

		#-------- for averaging over time --------
		# vR1s = np.zeros((1, 50))
		# vL1s = np.zeros((1, 50))
		# vR2s = np.zeros((1, 50))
		# vL2s = np.zeros((1, 50))
		# vR3s = np.zeros((1, 50))
		# vL3s = np.zeros((1, 50))

		# i = 0
		# for i in range(0, 50):
		# 	[v_R1, v_L1, v_R2, v_L2, v_R3, v_L3] = b_i.getCalibVals()
		# 	vR1s[0][i] = v_R1
		# 	vL1s[0][i] = v_L1
		# 	vR2s[0][i] = v_R2
		# 	vL2s[0][i] = v_L2
		# 	vR3s[0][i] = v_R3
		# 	vL3s[0][i] = v_L3
		# 	time.sleep(.25)
		# vR1 = np.sum(vR1s) / vR1s.size
		# vL1 = np.sum(vL1s) / vL1s.size
		# vR2 = np.sum(vR2s) / vR2s.size
		# vL2 = np.sum(vL2s) / vL2s.size
		# vR3 = np.sum(vR3s) / vR3s.size
		# vL3 = np.sum(vL3s) / vL3s.size
		# print(vR1s)
		#------------------------------------------

		#-------- for just one measurement --------
		[vR1, vL1, vR2, vL2, vR3, vL3] = b_i.getCalibVals()
		#------------------------------------------

		# Convert voltage to weight
		v = [vR1, vL1, vR2, vL2, vR3, vL3]
		A = [-1.0883, -1.0559, -1.1294, -1.1070, -1.0853, -1.1272]
		f = [A[i] * v[i] for i in range(len(v))]
		[R1, L1, R2, L2, R3, L3] = f

		# Jacobians
		N = np.array([[R1], [L1], [R2], [L2], [R3], [L3]])  # axial forces
		F = np.dot(J1, N)  # F = [Fx1; Fy1; Fz1; Mz1; Fx2; Fy2; Fz2; Mz2; Fx3; Fy3; Fz3; Mz3]
		# F = (1 / np.cos(t)) * F_ang  # adjust for force at an angle - NOOO WRONG BAD
		L = np.dot(J2, F)
		LL = np.dot(J3, L)
		output[n][m] = LL[tested][0]
		# print(F)
		print(LL)
	fs[n] = f

# print(fs)
np.savetxt(name, output, delimiter=',', comments='')
np.savetxt(name2, fs, delimiter=',', comments='')
