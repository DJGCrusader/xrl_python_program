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
# [[right hip, right knee, right ankle], [left hip, left knee, left ankle]]
usb_serials = [['367333693037', '375F366E3137', '366933693037'], ['376136583137', '366E33683037', '366933683037']]
# Find a connected ODrive (this will block until you connect one)
for leg in range(len(odrvs)):
    for joint in range(len(odrvs[0])):
        if usb_serials[leg][joint] == None:
            continue
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
    state = 'home'
    tStart = time.time()
    commAll()
    print('-----------------Begin')
    print('-------------------------------State: ',state)
    while(isRunning):
        time.sleep(0.010)
        t = time.time() - tStart
        if(state == 'home'): # homing sequence
            # send desired home pose
            rampTime = 10
            #thtDesired = zeroVec
            velDesired = zeroVec
            thtVals = xrlk.FrontalIK(0,53.7*in2m)
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = -thtVals[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = thtVals[i][j]+offsets[i][j][1]
            # ramp up kD and kP to high for some time
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    for k in range(0,2):
                        kD[i][j][k] = (t/rampTime)*kDd[i][j][k]
                        kP[i][j][k] = (t/rampTime)*kPd[i][j][k]
            # if near home pose or if ramptime is complete, change to idle state.
            if(t>=rampTime):
                #state = 'squatdown'
                #state = 'waitforsquat'
                state = 'holdZero'
                print('-------------------------------State: ',state)
                tStartSquat = t
        elif(state == 'holdZero'):
            state = 'holdZero'
            myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))
        elif(state == 'waitforsquat'):
            while True:
                i = input("Press Enter to continue or q+Enter to quit...")
                if not i:
                    break
                if i=='q':
                    cleanQuit()
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
            #IK, log all feedback over time
            rampTime = 10
            xDes = 0
            #For real robot: No lower than 32.75, midpoint is 42, max is 53.70
            yDes = (53.70-(((t - tStartSquat)/rampTime))*(53.70 - 53))*in2m
            thtVals = xrlk.FrontalIK(xDes,yDes)
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = -thtVals[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = thtVals[i][j]+offsets[i][j][1]
            myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))
            #print('Des:',niceList(thtDesired))
            #if reach low position, stand back up
            if(t - tStartSquat>= rampTime):
                tStartUp = t
                state = 'standup'
                print('-------------------------------State: ',state)
        elif(state == 'standup'): # stand up over period of time
            #IK, log all feedback over time
            rampTime = 10
            xDes = 0
            #For real robot: No lower than 32.75, midpoint is 42, max is 53.70
            yDes = (53.70-(1-((t - tStartUp)/rampTime))*(53.70 - 53))*in2m
            thtVals = xrlk.FrontalIK(xDes,yDes)
            #print(thtVals)
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = -thtVals[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = thtVals[i][j]+offsets[i][j][1]
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
    for leg in len(odrvs):
        for joint in len(odrvs[0]):
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
    #if control_mode == pos_control:
        #TODO do math so that thtDesired is correct format for odrvs

    ### Send Commands
    odrvs[leg][joint].motor0.set_pos_setpoint(thtDesired[leg][joint][0])
    odrvs[leg][joint].motor1.set_pos_setpoint(thtDesired[leg][joint][1])
    #if control_mode == vel_control:
    odrvs[leg][joint].motor0.set_vel_setpoint(velDesired[leg][joint][0])
    odrvs[leg][joint].motor1.set_vel_setpoint(velDesired[leg][joint][1])
    #update thtActual and velActual


    ### Read Current States
    thtActual[leg][joint][0] = odrvs[leg][joint].motor0.pos_setpoint
    thtActual[leg][joint][1] = odrvs[leg][joint].motor1.pos_setpoint
    velActual[leg][joint][0] = odrvs[leg][joint].motor0.vel_setpoint
    velActual[leg][joint][1] = odrvs[leg][joint].motor1.vel_setpoint
    #current_setpoint is probably incorrect
    curCommand[leg][joint][0] = odrvs[leg][joint].motor0.current_setpoint
    curCommand[leg][joint][1] = odrvs[leg][joint].motor1.current_setpoint

    '''
    TODO This is a part that needs to get translated into odrivetool functions:

    sendVals = struct.pack('dddddddd',\
                             thtDesired[leg][joint][0],\
                             thtDesired[leg][joint][1],\
                             velDesired[leg][joint][0],\
                             velDesired[leg][joint][1],\
                             kP[leg][joint][0],\
                             kP[leg][joint][1],\
                             kD[leg][joint][0],\
                             kD[leg][joint][1])
    msgOut = b'E'+sendVals+b'\n'
    '''


'''
def doComm2(leg,joint):
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand
    sendVals = struct.pack('dddddddd',\
                             thtDesired[leg][joint][0],\
                             thtDesired[leg][joint][1],\
                             velDesired[leg][joint][0],\
                             velDesired[leg][joint][1],\
                             kP[leg][joint][0],\
                             kP[leg][joint][1],\
                             kD[leg][joint][0],\
                             kD[leg][joint][1])
    msgOut = b'E'+sendVals+b'\n'
    #print(msgOut) #for debug
    ss[leg][joint].write(msgOut)
    #time.sleep(0.000005)
    msg = ss[leg][joint].read(1)
    #print('Msg1: ',msg) #fordebug
    if(msg!=b'S' or msg!=b'T'):
        if(msg==b'b'):
            msg = ss[leg][joint].readline()
            print(str(msg))
        msg = ss[leg][joint].read(1)
    #print('Msg2: ',msg) #for debug
    if(msg == b'S' or msg == b'T'):
        if msg == b'S':
            msg = ss[leg][joint].read(1)
        msg = ss[leg][joint].read(6*8+1)
        if(len(msg) == 6*8+1):
            thtActual[leg][joint][0] = struct.unpack('d', msg[0:8])[0]
            thtActual[leg][joint][1] = struct.unpack('d', msg[8:16])[0]
            velActual[leg][joint][0] = struct.unpack('d', msg[16:24])[0]
            velActual[leg][joint][1] = struct.unpack('d', msg[24:32])[0]
            curCommand[leg][joint][0] = struct.unpack('d', msg[32:40])[0]
            curCommand[leg][joint][1] = struct.unpack('d', msg[40:48])[0]
        else:
            print(msg,'\n', len(msg))
    else:
        print('From ',str(leg),str(joint),'Not S: ',msg,'\n', len(msg))
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
            odrvs[leg][joint].axis0.encoder.set_pll_bandwidth(157)
            odrvs[leg][joint].axis1.encoder.set_pll_bandwidth(157) 

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


signal.signal(signal.SIGINT, cleanQuitInt)

full_init(reset = True)
time.sleep(5)
printErrorStates()

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
"""`

#odrive.utils.start_liveplotter(lambda:[(odrvs[leg][joint].axis0.encoder.pos_estimate,odrvs[leg][joint].axis0.encoder.pos_estimate) for leg in range(len(odrvs)) for joint in range(len(odrvs[0]))])
#start_liveplotter(lambda:[odrv0.axis0.encoder.pos_estimate, odrv0.axis1.encoder.pos_estimate,odrv1.axis0.encoder.pos_estimate, odrv1.axis1.encoder.pos_estimate,odrv2.axis0.encoder.pos_estimate, odrv2.axis1.encoder.pos_estimate,odrv3.axis0.encoder.pos_estimate, odrv3.axis1.encoder.pos_estimate,odrv4.axis0.encoder.pos_estimate, odrv4.axis1.encoder.pos_estimate,odrv5.axis0.encoder.pos_estimate, odrv5.axis1.encoder.pos_estimate])

print("Done with program.")

#main(
