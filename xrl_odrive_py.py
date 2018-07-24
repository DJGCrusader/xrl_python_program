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
import xrl_odrive as xrlo

in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000

zeroVec = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
#offsets = [[[-8.59,-6.11],[-3.61,5.89],[4.03,0.21]],[[7.92,-0.77],[5.73,3.45],[-2.10,6.41]]]
offsets = zeroVec
thtDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
velDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
kP = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
kD = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
#kP = [[[10,10]] * 3] * 2
#kD = [[[0.5,0.5]] * 3] * 2
#kPd = [[[200,200],[200,200],[400,400]],[[200,200], [200,200], [400,400]]]
kPd = [[[10,10]] * 3] * 2
kDd = [[[0.5,0.5]] * 3] * 2
#kPd = [[[10,10],[200,200],[600,600]],[[100,100], [200,200], [600,600]]]
#kDd = [[[0.5,0.5],[15,15],[20,20]],[[10,10], [15,15], [20,20]]]
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
# [[right hip, right knee, right ankle], [left hip, left knee, left ankle]]

usb_serials = [['367333693037', '375F366E3137', '366933693037'], ['376136583137', '366E33683037', '366933683037']]
#usb_serials = [[None, '375F366E3137', None], [None, None, None]]
#usb_serials = [['367333693037', None, None], [None, None, None]]
#usb_serials = [[None, None, None], ['376136583137', None, None]]
#usb_serials = [[None, None, None], ['376136583137', '366E33683037', '366933683037']]
#usb_serials = [[None, None, None], [None, None, None]]
# Find a connected ODrive (this will block until you connect one)

def connect_all():
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            # if not connecting to one of the odrives, will just pass over it
            # similar lines throughout program
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

def main():
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand, t

    #connect to all odrives
    connect_all()
    xrlo.odrvs = odrvs

    isRunning = True
    t = 0

    ### LOGGER
    myLogger.appendData('\n--NewTrial--\n')
    titleStr = 't,'
    for i in range(0,len(odrvs)):
        for j in range(0,len(odrvs[0])):
            for k in range(0,2):
                titleStr+='thtDesired'+str(i)+str(j)+str(k)+','
    for i in range(0,len(odrvs)):
        for j in range(0,len(odrvs[0])):
            for k in range(0,2):
                titleStr+='thtActual'+str(i)+str(j)+str(k)+','
    for i in range(0,len(odrvs)):
        for j in range(0,len(odrvs[0])):
            for k in range(0,2):
                titleStr+='cmd'+str(i)+str(j)+str(k)+','
    myLogger.appendData(titleStr)

    ###INITIAL STATE
    state = 'home'


    ###SET CONTROL MODES
    #ramp gains to close to zero first
    #xrlo.ramp_to_very_low_gains()
    #set control modes
    xrlo.mixed_config_all()
    xrlo.closed_loop_state_all()
    ### SET THE MIXED GAINS
    xrlo.ramp_up_gains_all_sagittal(10, 1.0) #use cpr2rad if NOT in mixed mode
    xrlo.ramp_up_gains_all_frontal(10 ,1.0)
    xrlo.set_gear_ratios()
    #xrlo.ramp_up_gains_all_frontal(40, 0.5)



    tStart = time.time()
    commAll()
    print('-----------------Begin')
    print('-------------------------------State: ',state)
    while(isRunning):
        time.sleep(0.010)
        t = time.time() - tStart
        if(state == 'home'): # homing sequence
            # send desired home pose
            rampTime = 5
            #thtDesired = zeroVec
            velDesired = zeroVec
            #set thtDesired to standing position to start
            thtVals = xrlk.FrontalIK(0,53.7*in2m) #changed to rad
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = thtVals[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = 0 #thtVals[i][j]+offsets[i][j][1]

            #TODO - actually ramp up to the standing gains automatically
            ### this currently does nothing
            # ramp up kD and kP to high for some time
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    for k in range(0,2):
                        kD[i][j][k] = (t/rampTime)*kDd[i][j][k]
                        kP[i][j][k] = (t/rampTime)*kPd[i][j][k]
            print("Home")
            # if near home pose or if ramptime is complete, change to idle state.
            if(t>=rampTime):
                #state = 'squatdown'
                #state = 'waitforsquat'
                #state = 'holdZero'
                state = 'configure'
                print('-------------------------------State: ',state)
                tStartSquat = t
        elif(state == 'holdZero'):
            state = 'holdZero'
            myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))
        elif(state =='configure'):
            while True:
                i = input("Press q+Enter to quit or one of the following to configure:\npll\ngear ratio\ntorque constant\nmax gains all\nramp up gains\nramp up gains all\nreboot\nreboot all\nprint all\nprint errors\nprint gains\nprint pos\nprint desired tht\ninits\ntests\nsquat...")
                #if not i:
                #    break
                if i=='q':
                    cleanQuit()
                elif i=='squat':
                    state = 'waitforsquat'
                    break
                elif i=='tests':
                    state = 'tests'
                    break
                elif i=='inits':
                    state = 'inits'
                    break
                elif i=='max gains all':
                    xrlo.max_gains_all()
                elif i=='ramp up gains':
                    #get parameters
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    #gear = get_bool_from_user('geared')
                    s_kp = get_float_num_from_user('sagittal kp', 0.0, 500.0)
                    s_kd = get_float_num_from_user('sagittal kd', 0.0, 20.0)
                    f_kp = get_float_num_from_user('frontal kp', 0.0, 500.0)
                    f_kd = get_float_num_from_user('frontal kd', 0.0, 20.0)
                    xrlo.ramp_up_gains(leg, joint, s_kp, s_kd, f_kp, f_kd, rampSec=5, hz=100, debug=False)
                elif i=='ramp up gains all':
                    #gear = get_bool_from_user('geared')
                    #get parameters
                    s_kp = get_float_num_from_user('sagittal kp', 0.0, 500.0)
                    s_kd = get_float_num_from_user('sagittal kd', 0.0, 20.0)
                    f_kp = get_float_num_from_user('frontal kp', 0.0, 500.0)
                    f_kd = get_float_num_from_user('frontal kd', 0.0, 20.0)

                    xrlo.ramp_up_gains_all_sagittal(s_kp, s_kd, rampSec=5, hz=100, debug=False)
                    xrlo.ramp_up_gains_all_frontal(f_kp,f_kd, rampSec=5, hz=100, debug=False)
                elif i=='pll':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    pll_bandwidth = get_int_num_from_user('pll', range(10000))
                    xrlo.set_pll(leg, joint, pll_bandwidth)
                elif i=='gear ratio':
                    #leg = get_int_num_from_user('leg', range(2))
                    #joint = get_int_num_from_user('joint', range(3))
                    #motor = get_int_num_from_user('motor', range(2))
                    hip_ratio = get_float_num_from_user('hip gear ratio', 0, 100)
                    knee_ratio = get_float_num_from_user('knee gear ratio', 0, 100)
                    ankle_ratio = get_float_num_from_user('ankle gear ratio', 0, 100)

                    #gear_ratio = get_float_num_from_user('gear_ratio', 0, 100)
                    xrlo.gear_ratios_all(hip_ratio, knee_ratio, ankle_ratio)
                elif i=='torque constant':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    motor = get_int_num_from_user('motor', range(2))
                    torque_constant = get_float_num_from_user('torque_constant', 0, 100)
                    xrlo.set_torque_constant(torque_constant, leg, joint, motor)
                elif i=='print all':
                    print(odrvs)
                elif i=='print errors':
                    xrlo.printErrorStates()
                elif i=='print gains':
                    print("Sagittal kp: " + str(xrlo.get_sagittal_kp_gains_all()))
                    print("Sagittal kd: " + str(xrlo.get_sagittal_kd_gains_all()))
                    print("Frontal kp: " + str(xrlo.get_frontal_kp_gains_all()))
                    print("Frontal kd: " + str(xrlo.get_frontal_kd_gains_all()))
                elif i=='print pos':
                    print("Positions: " + str(xrlo.get_pos_all()))
                elif i=='print desired tht':
                    print(thtDesired)
                elif i=='reboot':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    xrlo.reboot(leg, joint)
                    connect_one(leg, joint)
                    xrlo.odrvs = odrvs
                elif i=='reboot all':
                    xrlo.reboot_all()
                    connect_all()
                    xrlo.odrvs = odrvs
        elif(state =='tests'):
            while True:
                i = input("Press Enter to return, q+Enter to quit, or one of the following to configure:\nposition\nposition all\nramptest\nramptest all...")
                if not i:
                    state = 'configure'
                    break
                if i=='q':
                    cleanQuit()
                elif i=='position':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    seconds = get_int_num_from_user('seconds', range(1000))
                    rads = get_float_num_from_user('rads', -100, 100)
                    xrlo.spin_to_pos(leg, joint, seconds, rads)
                elif i=='position all':
                    seconds = get_int_num_from_user('seconds', range(1000))
                    rads = get_float_num_from_user('rads', -100, 100)
                    xrlo.spin_to_pos_all(seconds, rads)
                elif i=='ramptest':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    seconds = get_int_num_from_user('seconds', range(1000))
                    rads = get_float_num_from_user('rads', -100, 100)
                    xrlo.ramp_test(leg, joint, seconds, rads)
                elif i=='ramptest all':
                    seconds = get_int_num_from_user('seconds', range(1000))
                    rads = get_float_num_from_user('rads', -100, 100)
                    xrlo.ramp_test_all(seconds, rads)
        elif(state =='inits'):
            while True:
                i = input("Press Enter to return, q+Enter to quit, or one of the following to configure:\ninit all\ninit joint\ninit motor\nmake perm\nmake perm all...")
                if not i:
                    state = 'configure'
                    break
                if i=='q':
                    cleanQuit()
                elif i=='init all':
                    reset = get_bool_from_user('reset')
                    xrlo.full_init(reset=reset)
                elif i=='init joint':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    reset = get_bool_from_user('reset')
                    xrlo.joint_init(leg, joint, reset=reset)
                elif i=='init motor':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    motor = get_int_num_from_user('motor', range(2))
                    reset = get_bool_from_user('reset')
                    print("leg: " + str(leg) + ", joint: " + str(joint) + ", motor: " + str(motor))
                    xrlo.single_init(leg,joint,motor, reset=reset)
                elif i=='make perm':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    xrlo.make_perm(leg, joint)
                elif i=='make perm all':
                    xrlo.make_perm_all()
        elif(state == 'waitforsquat'):
            while True:
                i = input("Press Enter to continue, or exit, configure, or q+Enter to quit...")
                if not i:
                    break
                if i=='configure':
                    state = 'configure'
                    break
                if i=='exit':
                    state = 'home'
                    break
                if i=='q':
                    cleanQuit()
            if state != 'configure':
                print("Squatting in 3...")
                time.sleep(1)
                print("             2...")
                time.sleep(1)
                print("             1...")
                time.sleep(1)
                state = 'squatdown'
                print('-------------------------------State: ',state)
                t = time.time() - tStart
                tStartSquat = t
        elif(state == 'idle'): # wait for user command to perform a squat
            # if user hits enter?
            state = 'idle'
            # if user hits q
            #state = 'quit'
        elif(state == 'squatdown' ): # squat down over period of time
            #TODO - enter time, height
            #IK, log all feedback over time
            rampTime = 20
            xDes = 0
            #For real robot: No lower than 32.75, midpoint is 42, max is 53.70
            yDes = (53.70-(((t - tStartSquat)/rampTime))*(53.70 - 33))*in2m
            thtVals = xrlk.FrontalIK(xDes,yDes)
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = thtVals[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = 0 #thtVals[i][j]+offsets[i][j][1]
            myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))
            #print('Des:',niceList(thtDesired))
            #if reach low position, stand back up
            if(t - tStartSquat>= rampTime):
                tStartUp = t
                state = 'standup'
                print('-------------------------------State: ',state)
        elif(state == 'standup'): # stand up over period of time
            #IK, log all feedback over time
            rampTime = 20
            xDes = 0
            #For real robot: No lower than 32.75, midpoint is 42, max is 53.70
            yDes = (53.70-(1-((t - tStartUp)/rampTime))*(53.70 - 33))*in2m
            thtVals = xrlk.FrontalIK(xDes,yDes)
            #print(thtVals)
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = thtVals[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = 0 #thtVals[i][j]+offsets[i][j][1]
            myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))
            #print('Des:',niceList(thtDesired))
            #If reach top, idle
            if(t - tStartUp>= rampTime):
                myLogger.writeOut()
                #state = 'idle'
                state = 'waitforsquat'
                #state = 'squatdown'
                #tStartSquat = t
                print('-------------------------------State: ',state)
        elif(state == 'quit'): # quit
            cleanQuit()
        commAll(state, t)

def commAll(state = '', t = 0):
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand
    for leg in range(len(odrvs)):
        for joint in range(len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrv_comm(leg, joint)
    print('Act:',niceList(thtActual))

def cleanQuit():
    global kP, kD, t, myLogger
    myLogger.writeOut()
    print("\n-----------------Interrupt received")
    isRunning = False
    print('-----------------Quitting...')
    print('-----------------Ramping Gains Down...')
    print('Time of End: ',t)
    for leg in range(0,len(odrvs)):
        for joint in range(0,len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].axis0.requested_state = AXIS_STATE_IDLE
            odrvs[leg][joint].axis1.requested_state = AXIS_STATE_IDLE

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
#    kP = zeroVec
#    kD = zeroVec
    commAll()
    print('-----------------Quit!')
    sys.exit(0)

def odrv_comm(leg, joint):
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand
    ### Send Commands

    ###Mixed Position Control
    #both, sagittal, frontal
    ### DEBUGGING
    print("leg: " + str(leg) + ", joint: " + str(joint) + ", thtDesired: " + str(thtDesired[leg][joint][0]) + ", thtActual: " + str(odrvs[leg][joint].axis0.encoder.pos_estimate * CPR2RAD))
    #odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, thtDesired[leg][joint][0], 0)

    ###Mixed Pos&Vel Control?
    #both, sagittal, frontal, sagittal_vel, frontal_vel
    odrvs[leg][joint].axis0.controller.set_mixed_setpoint(True, thtDesired[leg][joint][0], 0, velDesired[leg][joint][0], 0)

    ###Mixed Gains
    ###this currently is not working
    ###whole program is currently in a single set gain mode
    #no ability to change the gains in real time
    #works well for now but will probably need to be changed for final version
    #odrvs[leg][joint].axis0.controller.set_mixed_gains(True, kP[leg][joint][0], 0, kD[leg][joint][0], 0)


    '''
    ### Read Current States
    #position
    thtActual[leg][joint][0] = odrvs[leg][joint].axis0.encoder.pos_estimate * CPR2RAD
    thtActual[leg][joint][1] = odrvs[leg][joint].axis1.encoder.pos_estimate * CPR2RAD
    #velocity
    velActual[leg][joint][0] = odrvs[leg][joint].axis0.encoder.pll_vel
    velActual[leg][joint][1] = odrvs[leg][joint].axis1.encoder.pll_vel
    #current
    curCommand[leg][joint][0] = odrvs[leg][joint].axis0.motor.current_control.Iq_measured
    curCommand[leg][joint][1] = odrvs[leg][joint].axis1.motor.current_control.Iq_measured
    '''
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

            #axis state
            if(odrvs[leg][joint].axis0.motor.config.pre_calibrated == False):
                odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(1570)
                time.sleep(1)
                odrvs[leg][joint].axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            if(odrvs[leg][joint].axis1.motor.config.pre_calibrated == False):
                # Change velocity pll bandwidth high temporarily to ensure calibration works well
                odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(1570)
                time.sleep(1)
                odrvs[leg][joint].axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
                time.sleep(1)
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



def get_int_num_from_user(item_str, allowed_range):
    #use keyboard input to get a number from within an allowed range from the user
    result = None
    while True:
        #ask for input
        i = input("Press q+Enter to quit, or enter " + item_str + "...")
        #exit the function and cleanQuit
        if i=='q':
            cleanQuit()
            return None
        else: #otherwise if input exists try to parse it
            try:
                result = int(i)
            except:
                #if input is not a valid int
                print("invalid input, please try again")
                continue
            #if input not in the range
            if result not in allowed_range:
                print("outside allowed range, please try again")
                continue
            #otherwise return it
            else:
                return result

def get_float_num_from_user(item_str, lower_lim, upper_lim):
    result = None
    while True:
        #ask for input
        i = input("Press q+Enter to quit, or enter " + item_str + "...")
        #exit the function and cleanQuit
        if i=='q':
            cleanQuit()
            return None
        else: #otherwise if input exists try to parse it
            try:
                result = float(i)
            except:
                #if input is not a valid int
                print("invalid input, please try again")
                continue
            #if input not in the range
            if result < lower_lim or result > upper_lim:
                print("outside allowed range, please try again")
                continue
            #otherwise return it
            else:
                return result

def get_bool_from_user(item_str):
    while True:
        #ask for input
        i = input("Press q+Enter to quit, or enter t/f for bool " + item_str + "...")
        #exit the function and cleanQuit
        if i=='q':
            cleanQuit()
            return None
        else: #otherwise if input exists try to parse it
            if i =='t':
                return True
            if i =='f':
                return False
            else:
                print("invalid input, please try again")
                continue




signal.signal(signal.SIGINT, cleanQuitInt)




main()
# full_init(reset = True)
# time.sleep(5)
# printErrorStates()

'''
for leg in range(len(odrvs)):
    for joint in range(len(odrvs[0])):
        if odrvs[leg][joint] == None:
            continue
        xrlo.mixed_config_all(perm=True)
        #Note that mixed impedance control units are in radians already!
        odrvs[leg][joint].axis0.controller.config.pos_gain = 10*CPR2RAD #10 with no gearing is ok. 100 was violent. Need 450 to 900 max
        odrvs[leg][joint].axis1.controller.config.pos_gain = 10*CPR2RAD
        odrvs[leg][joint].axis0.controller.config.vel_gain = 0.75*CPR2RAD #1 with no gearing causes light shakes = 48/15 = 3.2Nms/rad per motor at joint. Nice! Need 2.177 overall
        odrvs[leg][joint].axis1.controller.config.vel_gain = 0.75*CPR2RAD
        odrvs[leg][joint].axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        odrvs[leg][joint].axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

'''


#odrive.utils.start_liveplotter(lambda:[(odrvs[leg][joint].axis0.encoder.pos_estimate,odrvs[leg][joint].axis0.encoder.pos_estimate) for leg in range(len(odrvs)) for joint in range(len(odrvs[0]))])
#start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis1.encoder.pos_estimate,odrv1.axis0.encoder.pos_estimate, odrv1.axis1.encoder.pos_estimate,odrv2.axis0.encoder.pos_estimate, odrv2.axis1.encoder.pos_estimate,odrv3.axis0.encoder.pos_estimate, odrv3.axis1.encoder.pos_estimate,odrv4.axis0.encoder.pos_estimate, odrv4.axis1.encoder.pos_estimate,odrv5.axis0.encoder.pos_estimate, odrv5.axis1.encoder.pos_estimate])

print("Done with program.")

#main(
