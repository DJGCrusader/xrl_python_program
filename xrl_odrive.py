
"""
Created on Thu Jul 18 15:19:02 2018

@author: beeman
"""

"""
helpful odrive functions for xrl state machine
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



odrvs = [[None, None, None], [None, None, None]]

'''[[right hip, right knee, right ankle], [left hip, left knee, left ankle]]'''
#0,0 = '367333693037'
usb_serials = [['367333693037', '375F366E3137', '366933693037'], ['376136583137', '366E33683037', '366933683037']]


def set_gear_ratios():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].axis0.controller.config.gear_ratio = gear_ratios[joint]
            odrvs[leg][joint].axis1.controller.config.gear_ratio = gear_ratios[joint]

def set_gear_ratios_all(hip, knee, ankle):
    gear_ratios_all = [hip, knee, ankle]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].axis0.controller.config.gear_ratio = gear_ratios_all[joint]
            odrvs[leg][joint].axis1.controller.config.gear_ratio = gear_ratios_all[joint]
def read_thts():
    thts = [[[0,0],[0,0],[0,0]],[[0,0],[0,0],[0,0]]]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            thts[leg][joint][0]  = odrvs[leg][joint].axis0.controller.theta_s
            thts[leg][joint][1]  = odrvs[leg][joint].axis0.controller.theta_f
    return thts



def print_controllers():
    gear_ratios_all = [hip, knee, ankle]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            print(odrvs[leg][joint].axis0.controller)
            print(odrvs[leg][joint].axis1.controller)

def printErrorStates():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            print('leg',leg, ' joint',joint, ' axis0 error:',bin(odrvs[leg][joint].axis0.error))
            print('leg',leg, ' joint',joint, ' axis1 error:',bin(odrvs[leg][joint].axis1.error))
            print('leg',leg, ' joint',joint, ' motor0 error:',bin(odrvs[leg][joint].axis0.motor.error))
            print('leg',leg, ' joint',joint, ' motor1 error:',bin(odrvs[leg][joint].axis1.motor.error))
            print('leg',leg, ' joint',joint, ' encoder0 error:',bin(odrvs[leg][joint].axis0.encoder.error))
            print('leg',leg, ' joint',joint, ' encoder1 error:',bin(odrvs[leg][joint].axis1.encoder.error))
            #print('leg',leg, ' joint',joint, ' axis0 debug:',odrvs[leg][joint].axis0.debug)
            #print('leg',leg, ' joint',joint, ' axis1 debug:',odrvs[leg][joint].axis1.debug)

def connect_all():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if usb_serials[leg][joint] == None:
                continue
            print("finding odrive: " + usb_serials[leg][joint] + "...")
            odrvs[leg][joint] = odrive.find_any(serial_number = usb_serials[leg][joint])
            print("found odrive! leg: " + str(leg) + ", joint: " + str(joint))

def full_init(reset = True):
    if(reset):
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False
                odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
                odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False

                odrvs[leg][joint].axis0.config.use_pll = True
                odrvs[leg][joint].axis1.config.use_pll = True
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
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
            if odrvs[leg][joint] == None:
                continue
            # Change velocity pll bandwidth back
            odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(314)
            odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(314)

            odrvs[leg][joint].axis0.set_estimator_vals(13, 135)
            odrvs[leg][joint].axis1.set_estimator_vals(13, 135)

            odrvs[leg][joint].axis0.config.use_pll = False
            odrvs[leg][joint].axis1.config.use_pll = False


            #motor and encoder pre_calibrated
            #if(odrvs[leg][joint].axis0.error == 0):
            odrvs[leg][joint].axis0.motor.config.pre_calibrated = True
            odrvs[leg][joint].axis0.encoder.config.pre_calibrated = True
            #else:
                #odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
                #odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False

            #if(odrvs[leg][joint].axis1.error == 0):
            odrvs[leg][joint].axis1.motor.config.pre_calibrated = True
            odrvs[leg][joint].axis1.encoder.config.pre_calibrated = True
            #else:
        #        odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
        #        odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False

            #gear ratio
            odrvs[leg][joint].axis0.controller.config.gear_ratio = 1#gear_ratios[joint]
            odrvs[leg][joint].axis1.controller.config.gear_ratio = 1#gear_ratios[joint]

            #gear ratio
            odrvs[leg][joint].axis0.controller.config.torque_constant = 0.45#Nm/A
            odrvs[leg][joint].axis1.controller.config.torque_constant = 0.45

            #should always be default on all - use these if gets screwed up ever
            #odrvs[leg][joint].axis0.controller.set_current_controller_bandwidth(157)
            #odrvs[leg][joint].axis1.controller.set_current_controller_bandwidth(157)

            #Control Mode
            odrvs[leg][joint].axis0.controller.config.control_mode = 5 #Switch to pure Impedance control
            odrvs[leg][joint].axis1.controller.config.control_mode = 5

            #Set closed loop gains
            odrvs[leg][joint].axis0.controller.config.pos_gain = 0.001
            odrvs[leg][joint].axis1.controller.config.pos_gain = 0.001
            odrvs[leg][joint].axis0.controller.config.vel_gain = 0.0001
            odrvs[leg][joint].axis1.controller.config.vel_gain = 0.0001

            #axis state
            odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            # save configuration
            odrvs[leg][joint].save_configuration()

    print("Done initializing!")

def joint_init(leg, joint,reset = False):
    if(reset):
        odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
        odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False
        odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
        odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False

        odrvs[leg][joint].axis0.config.use_pll = True
        odrvs[leg][joint].axis1.config.use_pll = True

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

    # Change velocity pll bandwidth back
    odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(314)
    odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(314)

    odrvs[leg][joint].axis0.set_estimator_vals(13, 135)
    odrvs[leg][joint].axis1.set_estimator_vals(13, 135)

    odrvs[leg][joint].axis0.config.use_pll = False
    odrvs[leg][joint].axis1.config.use_pll = False


    #motor and encoder pre_calibrated
    #if(odrvs[leg][joint].axis0.error == 0):
    odrvs[leg][joint].axis0.motor.config.pre_calibrated = True
    odrvs[leg][joint].axis0.encoder.config.pre_calibrated = True
    #else:
        #odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
        #odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False

    #if(odrvs[leg][joint].axis1.error == 0):
    odrvs[leg][joint].axis1.motor.config.pre_calibrated = True
    odrvs[leg][joint].axis1.encoder.config.pre_calibrated = True
    #else:
#        odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
#        odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False

    #gear ratio
    odrvs[leg][joint].axis0.controller.config.gear_ratio = 1#gear_ratios[joint]
    odrvs[leg][joint].axis1.controller.config.gear_ratio = 1#gear_ratios[joint]

    #gear ratio
    odrvs[leg][joint].axis0.controller.config.torque_constant = 0.45#Nm/A
    odrvs[leg][joint].axis1.controller.config.torque_constant = 0.45

    #should always be default on all - use these if gets screwed up ever
    #odrvs[leg][joint].axis0.controller.set_current_controller_bandwidth(157)
    #odrvs[leg][joint].axis1.controller.set_current_controller_bandwidth(157)

    #Control Mode
    odrvs[leg][joint].axis0.controller.config.control_mode = 5 #Switch to pure Impedance control
    odrvs[leg][joint].axis1.controller.config.control_mode = 5

    #Set closed loop gains
    odrvs[leg][joint].axis0.controller.config.pos_gain = 0.001
    odrvs[leg][joint].axis1.controller.config.pos_gain = 0.001
    odrvs[leg][joint].axis0.controller.config.vel_gain = 0.0001
    odrvs[leg][joint].axis1.controller.config.vel_gain = 0.0001

    #axis state
    odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    # save configuration
    odrvs[leg][joint].save_configuration()

    print("Done initializing!")



def make_perm(leg, joint):
    odrvs[leg][joint].save_configuration()

def make_perm_all():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].save_configuration()

def single_init(leg, joint, motor, reset = False):
    if odrvs[leg][joint] == None:
        return
    if (reset):
        if motor == 0:
            odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
            odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False
            odrvs[leg][joint].axis0.config.use_pll = True
        elif motor == 1:
            odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
            odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False
            odrvs[leg][joint].axis1.config.use_pll = True

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
        odrvs[leg][joint].axis0.set_estimator_vals(13, 135)
        odrvs[leg][joint].axis0.config.use_pll = False
    elif motor == 1:
        odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(314)
        odrvs[leg][joint].axis1.set_estimator_vals(13, 135)
        odrvs[leg][joint].axis1.config.use_pll = False
    printErrorStates()

    #motor and encoder pre_calibrated
    if motor == 0:
        #if(odrvs[leg][joint].axis0.error == 0):
        odrvs[leg][joint].axis0.motor.config.pre_calibrated = True
        odrvs[leg][joint].axis0.encoder.config.pre_calibrated = True
        #else:
    #        odrvs[leg][joint].axis0.motor.config.pre_calibrated = False
#            odrvs[leg][joint].axis0.encoder.config.pre_calibrated = False
    elif motor == 1:
        #if(odrvs[leg][joint].axis1.error == 0):
        odrvs[leg][joint].axis1.motor.config.pre_calibrated = True
        odrvs[leg][joint].axis1.encoder.config.pre_calibrated = True
        #else:
        #    odrvs[leg][joint].axis1.motor.config.pre_calibrated = False
        #    odrvs[leg][joint].axis1.encoder.config.pre_calibrated = False

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
        odrvs[leg][joint].axis0.controller.config.control_mode = 5 #Switch to pure Impedance control
    elif motor == 1:
        odrvs[leg][joint].axis1.controller.config.control_mode = 5

    #Set closed loop gains
    if motor == 0:
        odrvs[leg][joint].axis0.controller.config.pos_gain = 0.001
        odrvs[leg][joint].axis0.controller.config.vel_gain = 0.0001
    elif motor == 1:
        odrvs[leg][joint].axis1.controller.config.pos_gain = 0.001
        odrvs[leg][joint].axis1.controller.config.vel_gain = 0.0001

    #axis state
    if motor == 0:
        odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    elif motor == 1:
        odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

    # save configuration
    odrvs[leg][joint].save_configuration()

    print("Done initializing!")
def mixed_config_all(perm=False):
    #make sure saved gains are very low before turning on
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].axis0.controller.config.control_mode = 5
            odrvs[leg][joint].axis1.controller.config.control_mode = 5
            if(perm):
                odrvs[leg][joint].save_configuration()


def closed_loop_state_all(perm=False):
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            if(perm):
                odrvs[leg][joint].save_configuration()


def set_pll(leg, joint, pll_bandwidth, perm =False):
    odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(pll_bandwidth)
    odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(pll_bandwidth)
    if(perm):
        odrvs[leg][joint].save_configuration()
def set_gear_ratio(gear_ratio, leg, joint, motor):
    #gear ratio
    if motor == 0:
        odrvs[leg][joint].axis0.controller.config.gear_ratio = gear_ratio
    elif motor == 1:
        odrvs[leg][joint].axis1.controller.config.gear_ratio = gear_ratio

def set_torque_constant(torque_constant, leg, joint, motor):
    #gear ratio
    if motor == 0:
        odrvs[leg][joint].axis0.controller.config.torque_constant = torque_constant
    elif motor == 1:
        odrvs[leg][joint].axis1.controller.config.torque_constant = torque_constant

def ramp_to_very_low_gains(debug=False):
    if(debug):
        ramp_up_gains_all_sagittal(.001, 0.001,debug=True)
        ramp_up_gains_all_frontal(.001, 0.001,debug=True)
    else:
        ramp_up_gains_all_sagittal(.001, 0.001)
        ramp_up_gains_all_frontal(.001, 0.001)

def max_gains_all(rampSec=5, hz=100, debug=False):
    ramp_up_gains(0,0, 300, 2.5, 300, 2.5, rampSec=rampSec, hz=hz, debug=debug)
    ramp_up_gains(1,0, 300, 2.5, 300, 2.5, rampSec=rampSec, hz=hz, debug=debug)
    ramp_up_gains(0,1, 450, 3.7, 450, 3.7, rampSec=rampSec, hz=hz, debug=debug)
    ramp_up_gains(1,1, 450, 3.7, 450, 3.7, rampSec=rampSec, hz=hz, debug=debug)
    ramp_up_gains(0,2, 1000, 5, 2000, 5, rampSec=rampSec, hz=hz, debug=debug)
    ramp_up_gains(1,2, 1000, 5, 2000, 5, rampSec=rampSec, hz=hz, debug=debug)
    '''
    max_kp = [300, 450, 340]
    max_kd = [2.5, 3.7, 2.8]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            #print("leg: " + str(leg) + ", joint: " + str(joint))
            ramp_up_gains(leg, joint, max_kp[joint], max_kd[joint], max_kp[joint], max_kd[joint], rampSec=rampSec, hz=hz, debug=debug)
    '''

def ramp_up_gains(leg, joint, s_kp, s_kd, f_kp, f_kd, rampSec=5, hz=100, debug=False):
    if odrvs[leg][joint] == None:
        return
    start_s_kp = [odrvs[leg][joint].axis0.controller.config.pos_gain, odrvs[leg][joint].axis1.controller.config.pos_gain]
    start_s_kd = [odrvs[leg][joint].axis0.controller.config.vel_gain, odrvs[leg][joint].axis1.controller.config.vel_gain]
    start_f_kp = [odrvs[leg][joint].axis0.controller.config.pos_gain2, odrvs[leg][joint].axis1.controller.config.pos_gain2]
    start_f_kd = [odrvs[leg][joint].axis0.controller.config.vel_gain2, odrvs[leg][joint].axis1.controller.config.vel_gain2]
    if(debug):
        print("starting s_kp: " + str(start_s_kp))
        print("starting s_kd: " + str(start_s_kd))
        print("starting f_kp: " + str(start_f_kp))
        print("starting f_kd: " + str(start_f_kd))
    print("ramping gains to s_kp = " + str(s_kp) + ", s_kd = " + str(s_kd) + ", f_kp = " + str(f_kp)+ ", f_kd = " + str(f_kd)+ " on leg: " + str(leg) + ", joint: " + str(joint))
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

    print("done ramping gains")
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
            if odrvs[leg][joint] == None:
                continue
            start_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain, odrvs[leg][joint].axis1.controller.config.pos_gain]
            start_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain, odrvs[leg][joint].axis1.controller.config.vel_gain]

    if(debug):
        print("starting kp: " + str(start_kp))
        print("starting kd: " + str(start_kd))

    print("ramping gains to kp = " + str(kp) + ", kd = " + str(kd) + " on all")
    t_start = time.time()
    t = time.time() - t_start
    while (t < rampSec):
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                odrvs[leg][joint].axis0.controller.config.pos_gain = (1 - t/rampSec) * start_kp[leg][joint][0] + (t/rampSec) * kps[leg][joint]
                odrvs[leg][joint].axis1.controller.config.pos_gain = (1 - t/rampSec) * start_kp[leg][joint][1] + (t/rampSec) * kps[leg][joint]
                odrvs[leg][joint].axis0.controller.config.vel_gain = (1 - t/rampSec) * start_kd[leg][joint][0] + (t/rampSec) * kds[leg][joint]
                odrvs[leg][joint].axis1.controller.config.vel_gain = (1 - t/rampSec) * start_kd[leg][joint][1] + (t/rampSec) * kds[leg][joint]

        if(debug):
            cur_kp = [[None]*3]*2
            cur_kd = [[None]*3]*2
            for leg in range(len(odrvs)):
                for joint in range(len(odrvs[0])):
                    if odrvs[leg][joint] == None:
                        continue
                    cur_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain, odrvs[leg][joint].axis1.controller.config.pos_gain]
                    cur_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain, odrvs[leg][joint].axis1.controller.config.vel_gain]
            print("current kp: " + str(cur_kp))
            print("current kd: " + str(cur_kd))

        time.sleep(1/hz)
        t = time.time() - t_start
    print("done ramping gains to kp = " + str(kp) + ", kd = " + str(kd) + " on all")

    if(debug):
        end_kp = [[None]*3]*2
        end_kd = [[None]*3]*2
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                end_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain, odrvs[leg][joint].axis1.controller.config.pos_gain]
                end_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain, odrvs[leg][joint].axis1.controller.config.vel_gain]
        print("final kp: " + str(end_kp))
        print("final kd: " + str(end_kd))
    return True
def ramp_up_gains_all_frontal(kp, kd, rampSec=5, hz=100, debug=False):
    kps = [[kp]*3]*2
    kds = [[kd]*3]*2
    start_kp = [[None]*3]*2
    start_kd = [[None]*3]*2
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            start_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain2, odrvs[leg][joint].axis1.controller.config.pos_gain2]
            start_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain2, odrvs[leg][joint].axis1.controller.config.vel_gain2]
    if(debug):
        print("starting kp: " + str(start_kp))
        print("starting kd: " + str(start_kd))

    print("ramping gains to kp = " + str(kp) + ", kd = " + str(kd) + " on all")
    t_start = time.time()
    t = time.time() - t_start
    while (t < rampSec):
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                odrvs[leg][joint].axis0.controller.config.pos_gain2 = (1 - t/rampSec) * start_kp[leg][joint][0] + (t/rampSec) * kps[leg][joint]
                odrvs[leg][joint].axis1.controller.config.pos_gain2 = (1 - t/rampSec) * start_kp[leg][joint][1] + (t/rampSec) * kps[leg][joint]
                odrvs[leg][joint].axis0.controller.config.vel_gain2 = (1 - t/rampSec) * start_kd[leg][joint][0] + (t/rampSec) * kds[leg][joint]
                odrvs[leg][joint].axis1.controller.config.vel_gain2 = (1 - t/rampSec) * start_kd[leg][joint][1] + (t/rampSec) * kds[leg][joint]

        if(debug):
            cur_kp = [[None]*3]*2
            cur_kd = [[None]*3]*2
            for leg in range(len(odrvs)):
                for joint in range(len(odrvs[0])):
                    if odrvs[leg][joint] == None:
                        continue
                    cur_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain2, odrvs[leg][joint].axis1.controller.config.pos_gain2]
                    cur_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain2, odrvs[leg][joint].axis1.controller.config.vel_gain2]
            print("current kp: " + str(cur_kp))
            print("current kd: " + str(cur_kd))
        time.sleep(1/hz)
        t = time.time() - t_start
    print("done ramping gains to kp = " + str(kp) + ", kd = " + str(kd) + " on all")

    if(debug):
        end_kp = [[None]*3]*2
        end_kd = [[None]*3]*2
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                end_kp[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain2, odrvs[leg][joint].axis1.controller.config.pos_gain2]
                end_kd[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain2, odrvs[leg][joint].axis1.controller.config.vel_gain2]
        print("final kp: " + str(end_kp))
        print("final kd: " + str(end_kd))
    return True

def ramp_test_all(seconds, rads):
    start_pos = [[None, None, None],[None,None,None]]
    positions = [[None, None, None],[None,None,None]]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            start_pos[leg][joint] = [odrvs[leg][joint].axis0.encoder.pos_estimate, odrvs[leg][joint].axis1.encoder.pos_estimate]
    start_time = time.time()
    ramp_time = seconds/2
    print("beginning ramp test! total time: " + str(seconds) + ", total rads: " + str(rads))

    #spin up
    t = time.time() - start_time
    while (t < ramp_time):
        '''
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                positions[leg][joint] = [odrvs[leg][joint].axis0.encoder.pos_estimate, odrvs[leg][joint].axis1.encoder.pos_estimate]
        print("Positions: " + str(positions))
        '''
        #steadily increase up to rads from start_pos[0]
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue

                odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, (1-t/ramp_time)*start_pos[leg][joint][0]*CPR2RAD + (t/ramp_time)*rads, 0)
        time.sleep(0.01)
        t = time.time() - start_time
    print("reversing direction")
    #spin down
    t = time.time() - start_time - ramp_time
    while (t < ramp_time):
        '''
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                positions[leg][joint] = [odrvs[leg][joint].axis0.encoder.pos_estimate, odrvs[leg][joint].axis1.encoder.pos_estimate]
        print("Positions: " + str(positions))
        '''
        #steadily decrease down to start_pos[0] from rads
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, (1-t/ramp_time)*rads + (t/ramp_time)*start_pos[leg][joint][0]*CPR2RAD, 0)
        time.sleep(0.01)
        t = time.time() - start_time - ramp_time
    print("completed ramp test!")
def ramp_test(leg, joint, seconds, rads):
    start_pos = [odrvs[leg][joint].axis0.encoder.pos_estimate, odrvs[leg][joint].axis1.encoder.pos_estimate]
    start_time = time.time()
    ramp_time = seconds/2
    print("beginning ramp test! total time: " + str(seconds) + ", leg: " + str(leg) + ", joint: " + str(joint))
    #spin up
    t = time.time() - start_time
    while (t < ramp_time):
        #print(odrvs[leg][joint].axis0.encoder.pos_estimate)
        #steadily increase up to rads from start_pos[0]
        odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, (1-t/ramp_time)*start_pos[0]*CPR2RAD + (t/ramp_time)*rads, 0)
        time.sleep(0.01)
        t = time.time() - start_time
    print("reversing direction")
    #spin down
    t = time.time() - start_time - ramp_time
    while (t < ramp_time):
        #print(odrvs[leg][joint].axis0.encoder.pos_estimate)
        #steadily decrease down to start_pos[0] from rads
        odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, (1-t/ramp_time)*rads + (t/ramp_time)*start_pos[0]*CPR2RAD, 0)
        time.sleep(0.01)
        t = time.time() - start_time - ramp_time
    print("completed ramp test!")


def reboot(leg, joint):
    odrvs[leg][joint].reboot()

def reboot_all():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].reboot()

def spin_to_pos(leg, joint, seconds,rads):
    start_pos = [odrvs[leg][joint].axis0.encoder.pos_estimate, odrvs[leg][joint].axis1.encoder.pos_estimate]
    start_time = time.time()
    print("starting spin to zero, total time: " + str(seconds) + ", leg: " + str(leg) + ", joint: " + str(joint))
    #spin up
    t = time.time() - start_time
    while (t < seconds):
        print(odrvs[leg][joint].axis0.encoder.pos_estimate)
        #steadily increase up to rads from start_pos[0]
        odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, (1-t/seconds)*start_pos[0]*CPR2RAD + (t/seconds)*rads, 0)
        time.sleep(0.01)
        t = time.time() - start_time
    print("motor zeroed!")
def spin_to_pos_all(seconds,rads):
    start_pos = [[None, None, None],[None,None,None]]
    positions = [[None, None, None],[None,None,None]]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            start_pos[leg][joint] = [odrvs[leg][joint].axis0.encoder.pos_estimate, odrvs[leg][joint].axis1.encoder.pos_estimate]
    start_time = time.time()
    print("zeroing all motors, total time: " + str(seconds))

    #spin up
    t = time.time() - start_time
    while (t < seconds):
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                positions[leg][joint] = [odrvs[leg][joint].axis0.encoder.pos_estimate, odrvs[leg][joint].axis1.encoder.pos_estimate]
        print("Positions: " + str(positions))
        #steadily increase up to rads from start_pos[0]
        for leg in range(len(odrvs)):
            for joint in range(len(odrvs[0])):
                if odrvs[leg][joint] == None:
                    continue
                odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, (1-t/seconds)*start_pos[leg][joint][0]*CPR2RAD + (t/seconds)*rads, 0)
        time.sleep(0.01)
        t = time.time() - start_time
    print("done zeroing motors!")

def get_pos_all():
    positions = [[None,None,None],[None,None,None]]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            positions[leg][joint] = [odrvs[leg][joint].axis0.encoder.pos_estimate, odrvs[leg][joint].axis1.encoder.pos_estimate]
    return positions

def get_frontal_kp_gains_all():
    gains = [[None,None,None],[None,None,None]]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            gains[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain2,odrvs[leg][joint].axis1.controller.config.pos_gain2]
    return gains
def get_frontal_kd_gains_all():
    gains = [[None,None,None],[None,None,None]]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            gains[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain2,odrvs[leg][joint].axis1.controller.config.vel_gain2]
    return gains
def get_sagittal_kp_gains_all():
    gains = [[None,None,None],[None,None,None]]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            gains[leg][joint] = [odrvs[leg][joint].axis0.controller.config.pos_gain,odrvs[leg][joint].axis1.controller.config.pos_gain]
    return gains
def get_sagittal_kd_gains_all():
    gains = [[None,None,None],[None,None,None]]
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            gains[leg][joint] = [odrvs[leg][joint].axis0.controller.config.vel_gain,odrvs[leg][joint].axis1.controller.config.vel_gain]
    return gains
