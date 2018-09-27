import bridgeInterface as b_i
import time

"""
One-time Calibration
Run for each load cell (change S)
Put output into calibrator.m (yes I know this is inelegant)
calibrator.m gives you visualization + calibration constants mass = A*volts + B

"""
# --------WHICH ONE ARE YOU TESTING?---------
# R1 = 1
# L1 = 2
# R2 = 3
# L2 = 4
# R3 = 5
# L3 = 6
S = 4  # which load cell do you want to test?
# -------------------------------------------


b_i.initialize()
print("Hang 100g mass.")
time.sleep(7)
volts = b_i.getCalibVals()
m100v = volts[S - 1]
print(volts)

print("Hang 200g mass.")
time.sleep(12)
volts = b_i.getCalibVals()
m200v = volts[S - 1]
print(volts)

print("Hang 500g mass.")
time.sleep(12)
volts = b_i.getCalibVals()
m500v = volts[S - 1]

print("Hang 1000g mass.")
time.sleep(15)
volts = b_i.getCalibVals()
m1000v = volts[S - 1]
print(m100v, m200v, m500v, m1000v)
