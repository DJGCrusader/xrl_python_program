from __future__ import print_function
"""
Created on Thu Jun 28 16:34:02 2018

@author: beeman
"""
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

import odrive
from odrive.enums import *
import time
import math
import fibre

import serial
import struct
import signal
import sys
import xrl_kinematics as xrlk
from dataLogger import *

in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000

zeroVec = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
offsets = [[[-8.59,-6.11],[-3.61,5.89],[4.03,0.21]],[[7.92,-0.77],[5.73,3.45],[-2.10,6.41]]]
thtDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
velDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
kP = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
kD = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
#kPd = [[[200,200],[200,200],[400,400]],[[200,200], [200,200], [400,400]]]
kPd = [[[100,100],[200,200],[600,600]],[[100,100], [200,200], [600,600]]]
kDd = [[[10,10],[15,15],[20,20]],[[10,10], [15,15], [20,20]]]
#kPd = [[[20,20],[20,20],[20,20]],[[20,20], [20,20], [20,20]]]
#kDd = [[[2,2],[2,2],[2,2]],[[2,2], [2,2], [2,2]]]
#kPd = [[[10,10],[2,2],[2,2]],[[10,10], [2,2], [2,2]]]

gear_ratios = [32.0/15.0, 48.0/15.0, 36.0/15.0]
CPR2RAD = (2*math.pi/16384.0)
thtActual = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
velActual =   [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
curCommand = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
myLogger = dataLogger('data.txt')

#BAUD = 921600
#Robot facing 3DP:
#                   RIGHT LEG                    LEFT LEG
#ports = [['COM36', 'COM35', 'COM34'],['COM37', 'COM31', 'COM32']]
#ports = [['COM37', 'COM31', 'COM32']]
#ports = [['COM35', 'COM34'],['COM31','COM32']]  #for debug
#ports = [['COM36']]


odrvs = [[None, None, None], [None, None, None]]
## [[right hip, right knee, right ankle], [left hip, left knee, left ankle]]
usb_serials = [['367333693037', '375F366E3137', '366933693037'], ['376136583137', '366E33683037', '366933683037']]
#usb_serials = [[None, '375F366E3137', '366933693037'], ['376136583137', '366E33683037', '366933683037']]
# Find a connected ODrive (this will block until you connect one)
def connect_all():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if usb_serials[leg][joint] == None:
                continue
            print("finding odrive: " + usb_serials[leg][joint] + "...")
            odrvs[leg][joint] = odrive.find_any(serial_number = usb_serials[leg][joint])
            print("found odrive! leg: " + str(leg) + ", joint: " + str(joint))
def connect_one(leg, joint):
    print("finding odrive: " + usb_serials[leg][joint] + "...")
    odrvs[leg][joint] = odrive.find_any(serial_number = usb_serials[leg][joint])
    print("found odrive! leg: " + str(leg) + ", joint: " + str(joint))

isRunning = True
t = 0


'''
per odrive:
    odrvx.axisx.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    odrvx.axisx.motor.config.pre_calibrated = True
    odrvx.axisx.encoder.config.pre_calibrated = True
                       motor.config.current_lim = 50
odrvx.axisx.controller.config.gear_ratio = [ a different ]

save config
'''

def commAll(state = '', t = 0):
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand
    for leg in len(odrvs):
        for joint in len(odrvs[0]):
            odrv_comm(leg, joint)
    print('Act:',niceList(thtActual))
def odrv_comm(leg, joint):
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand
    #if control_mode == pos_control:
        #TODO do math so that thtDesired is correct format for odrvs

    ### Send Commands
    #set mixed setpoint, frontal = 0
    odrvs[leg][joint].axis0.controller.set_mixed_setpoint(True, thtDesired[leg][joint][0], 0)
    #set mixed gains, frontal = 0
    odrvs[leg][joint].axis0.controller.set_mixed_gains(True, kP[leg][joint][0], 0, kD[leg][joint], 0)


    ### Read Current States
    #position
    thtActual[leg][joint][0] = odrvs[leg][joint].motor0.pos_setpoint
    thtActual[leg][joint][1] = odrvs[leg][joint].motor1.pos_setpoint
    #velocity - vel_setpoint or pll_vel or vel_estimate?
    velActual[leg][joint][0] = odrvs[leg][joint].motor0.vel_estimate
    velActual[leg][joint][1] = odrvs[leg][joint].motor1.vel_estimate
    #current_setpoint is probably incorrect
    curCommand[leg][joint][0] = odrvs[leg][joint].motor0.current_setpoint
    curCommand[leg][joint][1] = odrvs[leg][joint].motor1.current_setpoint
def cleanQuit():
    global kP, kD, t, myLogger
    myLogger.writeOut()
    print("\n-----------------Interrupt received")
    isRunning = False
    print('-----------------Quitting...')
    print('-----------------Ramping Gains Down...')
    print('Time of End: ',t)
    t = time.time()
    tStart = t
    t = t-tStart
    rampTime = 2
    kDCurr = kD
    kPCurr = kP
    while(t < rampTime):
        time.sleep(0.010)
        t = time.time() - tStart
        for i in range(0,len(odrvs)):
            for j in range(0,len(odrvs[0])):
                #ss[i][j].flush()
                #not sure what to replace ^ with
                for k in range(0,2):
                    kD[i][j][k] = 0.1+(1-(t/rampTime))*(kDCurr[i][j][k]-0.1)
                    kP[i][j][k] = 1+(1-(t/rampTime))*(kPCurr[i][j][k]-1)
        commAll()
    print('-----------------All Gains at low')
    thtVals = xrlk.FrontalIK(0,53.7*in2m)
    for i in range(0,len(odrvs)):
        for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = -thtVals[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = thtVals[i][j]+offsets[i][j][1]
    #kP = zeroVec
    #kD = zeroVec
    commAll()
    print('-----------------Quit!')
    sys.exit(0)

def printErrorStates():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            print('leg',leg, ' joint',joint, ' axis0 error:',bin(odrvs[leg][joint].axis0.error))
            print('leg',leg, ' joint',joint, ' axis1 error:',bin(odrvs[leg][joint].axis1.error))
            print('leg',leg, ' joint',joint, ' motor0 error:',bin(odrvs[leg][joint].axis0.motor.error))
            print('leg',leg, ' joint',joint, ' motor1 error:',bin(odrvs[leg][joint].axis1.motor.error))
            print('leg',leg, ' joint',joint, ' encoder0 error:',bin(odrvs[leg][joint].axis0.encoder.error))
            print('leg',leg, ' joint',joint, ' encoder1 error:',bin(odrvs[leg][joint].axis1.encoder.error))
            print('leg',leg, ' joint',joint, ' axis0 debug:',odrvs[leg][joint].axis0.debug)
            print('leg',leg, ' joint',joint, ' axis1 debug:',odrvs[leg][joint].axis1.debug)
def cleanQuitInt(signal, frame):
    cleanQuit()
def niceList(theList):
    return ["%07.2f" % i for i in flattenList(theList)]
def flattenList(container):
    for i in container:
        if isinstance(i, (list,tuple)):
            for j in flattenList(i):
                yield j
        else:
            yield i

def full_init(reset = False):
    if(reset):
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False
                odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):

            #motor current limit
            odrvs[leg][joint].axis0.motor.config.current_lim = 50
            odrvs[leg][joint].axis1.motor.config.current_lim = 50

            #Calibration Max Voltage
            odrvs[leg][joint].axis0.motor.config.resistance_calib_max_voltage = 4
            odrvs[leg][joint].axis1.motor.config.resistance_calib_max_voltage = 4

            #motor calibration current
            odrvs[leg][joint].axis0.motor.config.calibration_current = 15
            odrvs[leg][joint].axis1.motor.config.calibration_current = 15

            #brake resistance
            odrvs[leg][joint].config.brake_resistance = 0

            # Change velocity pll bandwidth high temporarily to ensure calibration works well
            odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(1570)
            odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(1570)

            time.sleep(1)

            #axis state
            if(odrvs[leg][joint].axis0.motor.config.pre_calibrated == False):
                odrvs[leg][joint].axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            if(odrvs[leg][joint].axis1.motor.config.pre_calibrated == False):
                odrvs[leg][joint].axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    print("Done doing setup.")
    time.sleep(20)
    print("Saving Configuration...")
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            # Change velocity pll bandwidth back
            odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(314)
            odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(314)

            #motor and encoder pre_calibrated
            if(odrvs[leg][joint].axis0.error == 0):
                odrvs[leg][joint].axis0.motor.config.pre_calibrated = True
                odrvs[leg][joint].axis0.encoder.config.pre_calibrated = True
            else:
                odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False

            if(odrvs[leg][joint].axis1.error == 0):
                odrvs[leg][joint].axis1.motor.config.pre_calibrated = True
                odrvs[leg][joint].axis1.encoder.config.pre_calibrated = True
            else:
                odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False

            #gear ratio
            odrvs[leg][joint].axis0.controller.config.gear_ratio = 1#gear_ratios[joint]
            odrvs[leg][joint].axis1.controller.config.gear_ratio = 1#gear_ratios[joint]

            #gear ratio
            odrvs[leg][joint].axis0.controller.config.torque_constant = 0.45#Nm/A
            odrvs[leg][joint].axis1.controller.config.torque_constant = 0.45

            #Control Mode
            odrvs[leg][joint].axis0.controller.config.control_mode = 4 #Switch to pure Impedance control
            odrvs[leg][joint].axis1.controller.config.control_mode = 4

            #Set closed loop gains
            odrvs[leg][joint].axis0.controller.config.pos_gain = 0.0005
            odrvs[leg][joint].axis1.controller.config.pos_gain = 0.0005
            odrvs[leg][joint].axis0.controller.config.vel_gain = 0.00005
            odrvs[leg][joint].axis1.controller.config.vel_gain = 0.00005

            #axis state
            odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            # save configuration
            odrvs[leg][joint].save_configuration()

    print("Done initializing!")
def single_init(leg, joint, motor, reset = False):
    if (reset):
        if motor == 0:
            odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
            odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False
        elif motor == 1:
            odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
            odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False
    #motor current limit
    if motor == 0:
        odrvs[leg][joint].axis0.motor.config.current_lim = 50
    elif motor == 1:
        odrvs[leg][joint].axis1.motor.config.current_lim = 50

    #Calibration Max Voltage
    if motor == 0:
        odrvs[leg][joint].axis0.motor.config.resistance_calib_max_voltage = 4
    elif motor == 1:
        odrvs[leg][joint].axis1.motor.config.resistance_calib_max_voltage = 4

    #motor calibration current
    if motor == 0:
        odrvs[leg][joint].axis0.motor.config.calibration_current = 15
    elif motor == 1:
        odrvs[leg][joint].axis1.motor.config.calibration_current = 15

    #brake resistance
    odrvs[leg][joint].config.brake_resistance = 0

    #axis state
    if motor == 0:
        if(odrvs[leg][joint].axis0.motor.config.pre_calibrated == False):
            odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(1570)
            time.sleep(1)
            odrvs[leg][joint].axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
    elif motor == 1:
        if(odrvs[leg][joint].axis1.motor.config.pre_calibrated == False):
            # Change velocity pll bandwidth high temporarily to ensure calibration works well
            odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(1570)
            time.sleep(1)
            odrvs[leg][joint].axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            time.sleep(1)



    print("Done doing setup.")
    time.sleep(20)
    print("Saving Configuration...")
    # Change velocity pll bandwidth back
    if motor == 0:
        odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(314)
    elif motor == 1:
        odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(314)

    #motor and encoder pre_calibrated
    if motor == 0:
        if(odrvs[leg][joint].axis0.error == 0):
            odrvs[leg][joint].axis0.motor.config.pre_calibrated = True
            odrvs[leg][joint].axis0.encoder.config.pre_calibrated = True
        else:
            odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
            odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False
    elif motor == 1:
        if(odrvs[leg][joint].axis1.error == 0):
            odrvs[leg][joint].axis1.motor.config.pre_calibrated = True
            odrvs[leg][joint].axis1.encoder.config.pre_calibrated = True
        else:
            odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
            odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False

    #gear ratio
    if motor == 0:
        odrvs[leg][joint].axis0.controller.config.gear_ratio = 1#gear_ratios[joint]
    elif motor == 1:
        odrvs[leg][joint].axis1.controller.config.gear_ratio = 1#gear_ratios[joint]

    #gear ratio
    if motor == 0:
        odrvs[leg][joint].axis0.controller.config.torque_constant = 0.45#Nm/A
    elif motor == 1:
        odrvs[leg][joint].axis1.controller.config.torque_constant = 0.45

    #Control Mode
    if motor == 0:
        odrvs[leg][joint].axis0.controller.config.control_mode = 4 #Switch to pure Impedance control
    elif motor == 1:
        odrvs[leg][joint].axis1.controller.config.control_mode = 4

    #Set closed loop gains
    if motor == 0:
        odrvs[leg][joint].axis0.controller.config.pos_gain = 0.0005
        odrvs[leg][joint].axis0.controller.config.vel_gain = 0.00005
    elif motor == 1:
        odrvs[leg][joint].axis1.controller.config.pos_gain = 0.0005
        odrvs[leg][joint].axis1.controller.config.vel_gain = 0.00005

    #axis state
    if motor == 0:
        odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    elif motor == 1:
        odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    # save configuration
    odrvs[leg][joint].save_configuration()

    print("Done initializing!")


def speed_test(num_tests, leg, joint):
    odrvs[leg][joint].axis0.controller.config.control_mode = 5
    odrvs[leg][joint].axis1.controller.config.control_mode = 5
    odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    results = [None] * num_tests
    #setpoint = 0

    sagittal_pos = 0
    sagittal_kp = 5
    sagittal_kd = 0.75

    frontal_pos = 0
    frontal_kp = 5
    frontal_kd = 0.75

    dummy = 0
    print(odrvs[leg][joint].axis0.encoder.pos_estimate)

    odrvs[leg][joint].axis0.controller.set_mixed_gains(True, sagittal_kp, frontal_kp, sagittal_kd, frontal_kd)

    for i in range(num_tests):
        print(i)
        if i%2 == 1:
            #setpoint -= 10000
            sagittal_pos -= 4
        else:
            #setpoint += 10000
            sagittal_pos += 4


        #odrvs[leg][joint].axis0.controller.pos_setpoint = setpoint

        prev_time = time.time()
        odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, sagittal_pos, frontal_pos)
        new_time = time.time()

        dummy = odrvs[leg][joint].axis0.encoder.pos_estimate
        print(odrvs[leg][joint].axis0.encoder.pos_estimate)

        results[i] = new_time - prev_time
        time.sleep(0.5)

    avg = 0
    for i in range(num_tests):
        avg += results[i]
    avg = avg / num_tests
    print(str(avg) + " ms")
    print(str(1/avg) + " hz")
def speed_test_all(num_tests): #outdated as of 7/18
    print("starting speed test all")
    print("individual tests")
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            print("individual test: " + str(leg) + "," + str(joint))
            speed_test(2, leg, joint)
    print("done with individual tests")
    time.sleep(20)
    results = [None] * num_tests

    sagittal_pos = 0
    sagittal_kp = 5
    sagittal_kd = 0.75

    frontal_pos = 0
    frontal_kp = 5
    frontal_kd = 0.75

    dummy = 0

    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].axis0.controller.set_mixed_gains(True, sagittal_kp, frontal_kp, sagittal_kd, frontal_kd)

    print("starting to run tests")
    for i in range(num_tests):
        print("running test #" + str(i))

        if i%2 == 1:
            sagittal_pos -= 2
        else:
            sagittal_pos += 2


        #prev_time = time.time()
        prev_time = time.time()
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, sagittal_pos, frontal_pos)
        #new_time = time.time()
        #results[i] = new_time - prev_time

        #time.sleep(1)


        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                print(odrvs[leg][joint].axis0.encoder.pos_estimate)
        new_time = time.time()
        results[i] = new_time - prev_time

        time.sleep(1)

    print("done running tests")
    avg = 0
    for i in range(num_tests):
        avg += results[i]
    avg = avg / num_tests
    print("tests results: ")
    print(str(avg) + " ms")
    print(str(1/avg) + " hz")

def ramp_test(seconds, leg, joint):
    odrvs[leg][joint].axis0.controller.config.control_mode = 5
    odrvs[leg][joint].axis1.controller.config.control_mode = 5
    odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    results = [None] * (seconds * 100)
    #setpoint = 0

    sagittal_pos = 0
    sagittal_kp = 5
    sagittal_kd = 0.75

    frontal_pos = 0
    frontal_kp = 5
    frontal_kd = 0.75

    dummy = 0
    print(odrvs[leg][joint].axis0.encoder.pos_estimate)

    odrvs[leg][joint].axis0.controller.set_mixed_gains(True, sagittal_kp, frontal_kp, sagittal_kd, frontal_kd)

    for i in range(seconds * 100):
        #print(i)
        if i < (seconds * 100)/2:
            sagittal_pos += 0.01
        else:
            sagittal_pos -= 0.01


        #odrvs[leg][joint].axis0.controller.pos_setpoint = setpoint

        prev_time = time.time()
        odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, sagittal_pos, frontal_pos)
        new_time = time.time()

        dummy = odrvs[leg][joint].axis0.encoder.pos_estimate
        print(odrvs[leg][joint].axis0.encoder.pos_estimate)

        results[i] = new_time - prev_time
        time.sleep(0.01)

    avg = 0
    for i in range(seconds * 100):
        avg += results[i]
    avg = avg / (seconds * 100)
    print(str(avg) + " ms")
    print(str(1/avg) + " hz")
def ramp_test_all(seconds):
    mixed_config_all()
    counts = seconds * 100
    results = [None] * (counts)

    sagittal_pos = 0
    frontal_pos = 0

    for i in range(counts):
        if i < (counts)/2:
            sagittal_pos += 0.01
        else:
            sagittal_pos -= 0.01

        prev_time = time.time()
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, sagittal_pos, frontal_pos)
        new_time = time.time()

        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                print(odrvs[leg][joint].axis0.encoder.pos_estimate)

        results[i] = new_time - prev_time
        time.sleep(0.01)

    avg = 0
    for i in range(counts):
        avg += results[i]
    avg = avg / (counts)
    print(str(avg) + " ms")
    print(str(1/avg) + " hz")


def slow_spin_to_zero_all(seconds):
    mixed_config_all()

    start_pos = [[None] * 3] * 2
    command_pos = [[None] * 3] * 2
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            start_pos[leg][joint] = [odrvs[leg][joint].axis0.encoder.pos_estimate * CPR2RAD, odrvs[leg][joint].axis1.encoder.pos_estimate * CPR2RAD]
            #command_pos[leg][joint] = [odrvs[leg][joint].axis0.encoder.pos_estimate, odrvs[leg][joint].axis1.encoder.pos_estimate]
    print(start_pos)

    tStart = time.time()
    t = time.time() - tStart
    while (t < seconds):
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                command_pos[leg][joint] = (1 - t/seconds) * start_pos[leg][joint][0] + (t/seconds) * 0
                odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, command_pos[leg][joint], 0)
        time.sleep(0.01)
        t = time.time() - tStart
    print("back to 0!")



def ramp_up_gains(leg, joint, s_kp, s_kd, f_kp, f_kd, rampSec=5, hz=100, debug=False):
    start_s_kp = [odrvs[leg][joint].axis0.controller.config.pos_gain, odrvs[leg][joint].axis1.controller.config.pos_gain]
    start_s_kd = [odrvs[leg][joint].axis0.controller.config.vel_gain, odrvs[leg][joint].axis1.controller.config.vel_gain]
    start_f_kp = [odrvs[leg][joint].axis0.controller.config.pos_gain2, odrvs[leg][joint].axis1.controller.config.pos_gain2]
    start_f_kd = [odrvs[leg][joint].axis0.controller.config.vel_gain2, odrvs[leg][joint].axis1.controller.config.vel_gain2]
    if(debug):
        print("starting s_kp: " + str(start_s_kp))
        print("starting s_kd: " + str(start_s_kd))
        print("starting f_kp: " + str(start_f_kp))
        print("starting f_kd: " + str(start_f_kd))
    print("ramping up gains to s_kp = " + str(s_kp) + ", s_kd = " + str(s_kd) + ", f_kp = " + str(f_kp)+ ", f_kd = " + str(f_kd)+ " on leg: " + str(leg) + ", joint: " + str(joint))
    t_start = time.time()
    t = time.time() - t_start
    while (t < rampSec):
        #sagittal
        odrvs[leg][joint].axis0.controller.config.pos_gain = (1 - t/rampSec) * start_s_kp[0] + (t/rampSec) * s_kp
        odrvs[leg][joint].axis1.controller.config.pos_gain = (1 - t/rampSec) * start_s_kp[1] + (t/rampSec) * s_kp
        odrvs[leg][joint].axis0.controller.config.vel_gain = (1 - t/rampSec) * start_s_kd[0] + (t/rampSec) * s_kd
        odrvs[leg][joint].axis1.controller.config.vel_gain = (1 - t/rampSec) * start_s_kd[1] + (t/rampSec) * s_kd
        #frontal
        odrvs[leg][joint].axis0.controller.config.pos_gain2 = (1 - t/rampSec) * start_f_kp[0] + (t/rampSec) * f_kp
        odrvs[leg][joint].axis1.controller.config.pos_gain2 = (1 - t/rampSec) * start_f_kp[1] + (t/rampSec) * f_kp
        odrvs[leg][joint].axis0.controller.config.vel_gain2 = (1 - t/rampSec) * start_f_kd[0] + (t/rampSec) * f_kd
        odrvs[leg][joint].axis1.controller.config.vel_gain2 = (1 - t/rampSec) * start_f_kd[1] + (t/rampSec) * f_kd
        #wait
        time.sleep(1/hz)
        t = time.time() - t_start

        if(debug):
            print(t)
            print(odrvs[leg][joint].axis0.controller.config.pos_gain)
            print(odrvs[leg][joint].axis1.controller.config.pos_gain)
            print(odrvs[leg][joint].axis0.controller.config.vel_gain)
            print(odrvs[leg][joint].axis1.controller.config.vel_gain)
            print(odrvs[leg][joint].axis0.controller.config.pos_gain2)
            print(odrvs[leg][joint].axis1.controller.config.pos_gain2)
            print(odrvs[leg][joint].axis0.controller.config.vel_gain2)
            print(odrvs[leg][joint].axis1.controller.config.vel_gain2)

    print("done ramping up gains")
    if(debug):
        print("motor 0")
        print("s_kp = " + str(odrvs[leg][joint].axis0.controller.config.pos_gain) + ", s_kd = " + str(odrvs[leg][joint].axis0.controller.config.vel_gain))
        print("s_kp = " + str(odrvs[leg][joint].axis0.controller.config.pos_gain2) + ", s_kd = " + str(odrvs[leg][joint].axis0.controller.config.vel_gain2))
        print("motor 1")
        print("s_kp = " + str(odrvs[leg][joint].axis1.controller.config.pos_gain) + ", s_kd = " + str(odrvs[leg][joint].axis1.controller.config.vel_gain))
        print("s_kp = " + str(odrvs[leg][joint].axis1.controller.config.pos_gain2) + ", s_kd = " + str(odrvs[leg][joint].axis1.controller.config.vel_gain2))
def ramp_up_gains_all_sagittal(kp, kd, rampSec=5, hz=100, debug=False):
    kps = [[kp]*3]*2
    kds = [[kd]*3]*2
    start_kp = [[None]*3]*2
    start_kd = [[None]*3]*2
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            start_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain, odrvs[leg][joint].axis1.controller.config.pos_gain]
            start_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain, odrvs[leg][joint].axis1.controller.config.vel_gain]

    if(debug):
        print("starting kp: " + str(start_kp))
        print("starting kd: " + str(start_kd))

    print("ramping up gains to kp = " + str(kp) + ", kd = " + str(kd) + " on all")
    t_start = time.time()
    t = time.time() - t_start
    while (t < rampSec):
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                odrvs[leg][joint].axis0.controller.config.pos_gain = (1 - t/rampSec) * start_kp[leg][joint][0] + (t/rampSec) * kps[leg][joint]
                odrvs[leg][joint].axis1.controller.config.pos_gain = (1 - t/rampSec) * start_kp[leg][joint][1] + (t/rampSec) * kps[leg][joint]
                odrvs[leg][joint].axis0.controller.config.vel_gain = (1 - t/rampSec) * start_kd[leg][joint][0] + (t/rampSec) * kds[leg][joint]
                odrvs[leg][joint].axis1.controller.config.vel_gain = (1 - t/rampSec) * start_kd[leg][joint][1] + (t/rampSec) * kds[leg][joint]

        if(debug):
            cur_kp = [[None]*3]*2
            cur_kd = [[None]*3]*2
            for leg in range(len(odrvs)):
                for joint in range(len(odrvs[0])):
                    cur_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain, odrvs[leg][joint].axis1.controller.config.pos_gain]
                    cur_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain, odrvs[leg][joint].axis1.controller.config.vel_gain]
            print("current kp: " + str(cur_kp))
            print("current kd: " + str(cur_kd))

        time.sleep(1/hz)
        t = time.time() - t_start
    print("done ramping up gains to kp = " + str(kp) + ", kd = " + str(kd) + " on all")

    if(debug):
        end_kp = [[None]*3]*2
        end_kd = [[None]*3]*2
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                end_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain, odrvs[leg][joint].axis1.controller.config.pos_gain]
                end_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain, odrvs[leg][joint].axis1.controller.config.vel_gain]
        print("final kp: " + str(end_kp))
        print("final kd: " + str(end_kd))
def ramp_up_gains_all_frontal(kp, kd, rampSec=5, hz=100, debug=False):
    kps = [[kp]*3]*2
    kds = [[kd]*3]*2
    start_kp = [[None]*3]*2
    start_kd = [[None]*3]*2
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            start_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain2, odrvs[leg][joint].axis1.controller.config.pos_gain2]
            start_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain2, odrvs[leg][joint].axis1.controller.config.vel_gain2]
    if(debug):
        print("starting kp: " + str(start_kp))
        print("starting kd: " + str(start_kd))

    print("ramping up gains to kp = " + str(kp) + ", kd = " + str(kd) + " on all")
    t_start = time.time()
    t = time.time() - t_start
    while (t < rampSec):
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                odrvs[leg][joint].axis0.controller.config.pos_gain2 = (1 - t/rampSec) * start_kp[leg][joint][0] + (t/rampSec) * kps[leg][joint]
                odrvs[leg][joint].axis1.controller.config.pos_gain2 = (1 - t/rampSec) * start_kp[leg][joint][1] + (t/rampSec) * kps[leg][joint]
                odrvs[leg][joint].axis0.controller.config.vel_gain2 = (1 - t/rampSec) * start_kd[leg][joint][0] + (t/rampSec) * kds[leg][joint]
                odrvs[leg][joint].axis1.controller.config.vel_gain2 = (1 - t/rampSec) * start_kd[leg][joint][1] + (t/rampSec) * kds[leg][joint]

        if(debug):
            cur_kp = [[None]*3]*2
            cur_kd = [[None]*3]*2
            for leg in range(len(odrvs)):
                for joint in range(len(odrvs[0])):
                    cur_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain2, odrvs[leg][joint].axis1.controller.config.pos_gain2]
                    cur_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain2, odrvs[leg][joint].axis1.controller.config.vel_gain2]
            print("current kp: " + str(cur_kp))
            print("current kd: " + str(cur_kd))
        time.sleep(1/hz)
        t = time.time() - t_start
    print("done ramping up gains to kp = " + str(kp) + ", kd = " + str(kd) + " on all")

    if(debug):
        end_kp = [[None]*3]*2
        end_kd = [[None]*3]*2
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                end_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain2, odrvs[leg][joint].axis1.controller.config.pos_gain2]
                end_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain2, odrvs[leg][joint].axis1.controller.config.vel_gain2]
        print("final kp: " + str(end_kp))
        print("final kd: " + str(end_kd))

def mixed_config(leg, joint):
    odrvs[leg][joint].axis0.controller.config.control_mode = 5
    odrvs[leg][joint].axis1.controller.config.control_mode = 5
    odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
def mixed_config_all():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            odrvs[leg][joint].axis0.controller.config.control_mode = 5
            odrvs[leg][joint].axis1.controller.config.control_mode = 5
            odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

def set_plls_all(pll_bandwidth, perm=False):
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(pll_bandwidth)
            odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(pll_bandwidth)
            if (perm):
                odrvs[leg][joint].save_configuration()

def set_plls_joint(pll_bandwidth, leg, joint, perm =False):
    odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(pll_bandwidth)
    odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(pll_bandwidth)
    if(perm):
        odrvs[leg][joint].save_configuration()

def set_gear_ratios_motor(gear_ratio, torque_constant, leg, joint, motor):
    #gear ratio
    if motor == 0:
        odrvs[leg][joint].axis0.controller.config.gear_ratio = gear_ratio
    elif motor == 1:
        odrvs[leg][joint].axis1.controller.config.gear_ratio = gear_ratio
    #gear ratio
    if motor == 0:
        odrvs[leg][joint].axis0.controller.config.torque_constant = torque_constant
    elif motor == 1:
        odrvs[leg][joint].axis1.controller.config.torque_constant = torque_constant
def set_gear_ratios(gear_ratio, torque_constant, leg, joint):
    #gear ratio
    odrvs[leg][joint].axis0.controller.config.gear_ratio = gear_ratio
    odrvs[leg][joint].axis1.controller.config.gear_ratio = gear_ratio
    #gear ratio
    odrvs[leg][joint].axis0.controller.config.torque_constant = torque_constant
    odrvs[leg][joint].axis1.controller.config.torque_constant = torque_constant
def set_gear_ratios_all(gear_ratio, torque_constant):
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            set_gear_ratios(gear_ratio, torque_constant, leg, joint)


CPR2RAD = (2*math.pi/16384.0)

signal.signal(signal.SIGINT, cleanQuitInt)


'''
connect_one(1,2)
print(odrvs)
speed_test(20,1,2)

'''
print(odrive)
#connect_all()
#speed_test_all(10)

'''
connect_all()
for leg in range(len(odrvs)):
    for joint in range(len(odrvs[0])):
        print("testing leg: " + str(leg) + ", joint: " + str(joint))
        ramp_test(14, leg, joint)
        print("done testing leg: " + str(leg) + ", joint: " + str(joint))
        '''

#connect_one(0,0)
#ramp_test(14,0,0)


connect_all()
mixed_config_all()
ramp_up_gains_all_sagittal(1,0.1)
ramp_up_gains_all_frontal(40, 0.5)
ramp_test_all(14)
#ramp_test_all(14)
#slow_spin_to_zero_all(60)

#connect_one(0,0)
#single_init(0,0,1,True)
#mixed_config(0,0)
#ramp_up_gains(0,0,1,0.1,70,0.5,debug=True)
#ramp_test(14,0,0)

#ramp_test(14)
#single_init(0,0,1,True)
#single_init(0,1,0,True)
#single_init(0,1,1,True)
#single_init(0,2,0,True)
#single_init(1,0,1,True)
#single_init(0,2,0,True)

#single_init(0,2,1,True)
#connect_one(1,0)
#ramp_up_gains(1,0,40,0.5)

#speed_test()
#full_init(reset = True)
#time.sleep(5)
#printErrorStates()

"""
for leg in range(len(odrvs)):
    for joint in range(len(odrvs[0])):
        #Note that mixed impedance control units are in radians already!
        odrvs[leg][joint].axis0.controller.config.pos_gain = 1*CPR2RAD #Need 450 to 900 max
        odrvs[leg][joint].axis1.controller.config.pos_gain = 1*CPR2RAD
        odrvs[leg][joint].axis0.controller.config.vel_gain = 0*CPR2RAD #1.5 causes shakes. Need 2.177
        odrvs[leg][joint].axis1.controller.config.vel_gain = 0*CPR2RAD
        odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
"""


#odrive.utils.start_liveplotter(lambda:[(odrvs[leg][joint].axis0.encoder.pos_estimate,odrvs[leg][joint].axis0.encoder.pos_estimate) for leg in range(len(odrvs)) for joint in range(len(odrvs[0]))])

#start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis1.encoder.pos_estimate,odrv1.axis0.encoder.pos_estimate, odrv1.axis1.encoder.pos_estimate,odrv2.axis0.encoder.pos_estimate, odrv2.axis1.encoder.pos_estimate,odrv3.axis0.encoder.pos_estimate, odrv3.axis1.encoder.pos_estimate,odrv4.axis0.encoder.pos_estimate, odrv4.axis1.encoder.pos_estimate,odrv5.axis0.encoder.pos_estimate, odrv5.axis1.encoder.pos_estimate])
print("Done with program.")

#main()
