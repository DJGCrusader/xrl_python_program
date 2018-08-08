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
import datetime
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

#wednesday
#solving for jump problem

#wednesday
#offsets = [[[-0.02088712342083454, 0.12046219408512115], [-0.05574047565460205, -0.05557453632354736], [-0.004209842998534441, 0.03389998897910118]], [[0.038227371871471405, -0.1437968760728836], [0.0021709883585572243, 0.011283609084784985], [-0.12486334145069122, 0.11613021045923233]]]
#after lunch
#offsets = [[[-0.10940675437450409, 0.22007878124713898], [0.049144547432661057, -0.05353261157870293], [-0.025068538263440132, 0.041446980088949203]], [[0.0006983056082390249, -0.23880870640277863], [0.10245130211114883, 0.0029176988173276186], [-0.1488809436559677, 0.10424432158470154]]]
#12:50 Tuesday - "extremely stable"
#offsets = [[[-0.10957961529493332, 0.3974084258079529], [0.060073234140872955, -0.054044242948293686], [-0.030427640303969383, 0.04151458293199539]], [[-0.01245207991451025, -0.4045228958129883], [0.11808610707521439, 0.00739334337413311], [-0.14749814569950104, 0.10335318744182587]]]
#start of day Tuesday - hanging, after lots of ankle adjustments
#offsets = [[[-0.0949634537100792, 0.027227234095335007], [0.06307390332221985, -0.0366579033434391], [-0.05070863291621208, 0.0589132234454155]], [[-0.05666694790124893, -0.06815105676651001], [0.13809514045715332, 0.021299656480550766], [-0.15321984887123108, 0.06288952380418777]]]
#end of monday, lower position
#offsets = [[[0.025139199569821358, 0.4169404208660126], [-0.08885841816663742, -0.03869061544537544], [0.0006883249152451754, 0.02242586575448513]], [[0.09908418357372284, -0.4298003911972046], [-0.030209466814994812, 0.004397288896143436], [0.03991054370999336, -0.08211345225572586]]]
#after readjusting hip frontal and a lot of hijinks
#offsets = [[[-0.13014867901802063, 0.32747358083724976], [0.056090787053108215, -0.053993549197912216], [-0.04855147376656532, 0.04668930917978287]], [[-0.04312245920300484, -0.3384183645248413], [0.16965053975582123, -0.008075520396232605], [-0.03289823606610298, -0.06612218916416168]]]
#and again after adjusting ankles
#offsets = [[[-0.11737857013940811, 0.04512752592563629], [0.08082898706197739, -0.04386226460337639], [-0.04254092648625374, 0.02633456513285637]], [[-0.07544530183076859, -0.0757979229092598], [0.17157724499702454, -0.007803570944815874], [0.006379296071827412, -0.0792679637670517]]]
#third Time
#offsets = [[[-0.1255439668893814, 0.06385748088359833], [0.04843009635806084, -0.035445649176836014], [-0.004824417177587748, 0.037347760051488876]], [[-0.037003595381975174, -0.10268636792898178], [0.14225736260414124, 0.018547892570495605], [-0.01547501515597105, -0.09135666489601135]]]
#second hanging Time
#offsets = [[[-0.1134306788444519, 0.025913581252098083], [0.06536473333835602, -0.03975536301732063], [-0.03114054724574089, 0.019967561587691307]], [[-0.06635342538356781, -0.0607600212097168], [0.14181487262248993, 0.015657847747206688], [0.014098365791141987, -0.07556822150945663]]]
#original, from hanging position
#offsets = [[[-0.11466138064861298, 0.022034838795661926], [0.08220256865024567, -0.038824282586574554], [-0.037347760051488876, 0.020244121551513672]], [[-0.09730729460716248, -0.056528668850660324], [0.17874933779239655, 0.00012905863695777953], [0.009415296837687492, -0.06886934489011765]]]
# second version, probs wrong: offsets = [[[-0.003477729856967926, 0.005918363109230995], [-0.012491249479353428, -0.006582105066627264], [-0.023679599165916443, 0.0411519818007946]], [[0.011753757484257221, -0.020105842500925064], [-0.0028531677089631557, 0.004249791149049997], [0.03999043628573418, -0.08573944121599197]]]
home_thts = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
#SAGITTAL = 0 | FRONTAL = 1
home_kp = [[[300, 300], [450, 450], [1000, 2000]], [[300, 300], [450, 450], [1000, 2000]]]
home_kd = [[[2.5,2.5], [3.7, 3.7], [5,5]], [[2.5,2.5], [3.7, 3.7], [5,5]]]
thtDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
velDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
kPDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
kDDesired = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]

gear_ratios = [32.0/15.0, 48.0/15.0, 36.0/15.0]

CPR2RAD = (2*math.pi/16384.0)

thtActual = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
velActual =   [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
curCommand = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]

now = datetime.datetime.now()
myLoggerName = str(now.strftime("%Y-%m-%d__%H:%M")) + ".txt"
myLogger = dataLogger(myLoggerName)

seconds = 0
height = 0
max_height = 53.7241049455
wait_down = False
dynamic_gains = False
#BAUD = 921600

squat_home = xrlk.FrontalIK(0,max_height*in2m) #((-0.031870902996822714, 3.480880635731154e-06, 0.03186742211618698), (-0.031870902996822714, 3.480880635731154e-06, 0.03186742211618698))
squat_offsets = [[[-0.02088712342083454, 0.12046219408512115], [-0.05574047565460205, -0.05557453632354736], [-0.004209842998534441, 0.03389998897910118]], [[0.038227371871471405, -0.1437968760728836], [0.0021709883585572243, 0.011283609084784985], [-0.12486334145069122, 0.11613021045923233]]]
offsets = [[[-0.02088712342083454, 0.12046219408512115], [-0.05574047565460205, -0.05557453632354736], [-0.004209842998534441, 0.03389998897910118]], [[0.038227371871471405, -0.1437968760728836], [0.0021709883585572243, 0.011283609084784985], [-0.12486334145069122, 0.11613021045923233]]]
print("squat home")
print(squat_home)
print("original offsets")
print(offsets)
'''
for i in range(0,len(offsets)):
    for j in range(0,len(offsets[0])):
        offsets[i][j][1] += squat_home[i][j]
print("adjusted offsets")
print(offsets)
'''

# [[right hip, right knee, right ankle], [left hip, left knee, left ankle]]
odrvs = [[None, None, None], [None, None, None]]
usb_serials = [['367333693037', '375F366E3137', '366933693037'], ['376136583137', '366E33683037', '366933683037']]
#usb_serials = [[None, None, None], [None, None, None]]

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
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand, t, offsets
    global seconds, height, max_height, wait_down

    #connect to all odrives
    connect_all()
    xrlo.odrvs = odrvs

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
    xrlo.mixed_config_all()
    xrlo.closed_loop_state_all()

    ### SET THE MIXED GAINS
    xrlo.ramp_up_gains_all_sagittal(0, 0.0) #use cpr2rad if NOT in mixed mode
    xrlo.ramp_up_gains_all_frontal(0 ,0.0)
    xrlo.set_gear_ratios()

    isRunning = True
    t = 0
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
            thtVals = [[0,0,0],[0,0,0]]
            #thtVals = xrlk.FrontalIK(0,53.7*in2m) #changed to rad
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]

            #TODO - actually ramp up to the standing gains automatically
            ### this currently does nothing
            # ramp up kD and kP to high for some time
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    for k in range(0,2):
                        kDDesired[i][j][k] = (t/rampTime)*home_kd[i][j][k]
                        kPDesired[i][j][k] = (t/rampTime)*home_kp[i][j][k]
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
                i = input("Press q+Enter to quit or one of the following to configure:\ntorque constant\npll\ngear ratio\nprint errors\nprint gains\nprint pos\nprint all\nthts\ngains\ninits\ntests\ndynamic gains\nsquat...")
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
                elif i=='gains':
                    state = 'gains'
                    break
                elif i=='thts':
                    state = 'thts'
                    break
                elif i=='dynamic gains':
                    state = 'dynamic gains'
                    break

                ###configuartion parameters
                elif i=='gear ratio':
                    hip_ratio = get_float_num_from_user('hip gear ratio', 0, 100)
                    knee_ratio = get_float_num_from_user('knee gear ratio', 0, 100)
                    ankle_ratio = get_float_num_from_user('ankle gear ratio', 0, 100)
                    xrlo.gear_ratios_all(hip_ratio, knee_ratio, ankle_ratio)
                elif i=='torque constant':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    motor = get_int_num_from_user('motor', range(2))
                    torque_constant = get_float_num_from_user('torque_constant', 0, 100)
                    xrlo.set_torque_constant(torque_constant, leg, joint, motor)
                elif i=='pll':
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    pll_bandwidth = get_int_num_from_user('pll', range(10000))
                    xrlo.set_pll(leg, joint, pll_bandwidth)

                ###print commands
                elif i=='print all':
                    print(odrvs)
                elif i=='print errors':
                    xrlo.printErrorStates()
                elif i=='print gearing':
                    xrlo.print_controller()
                elif i=='print gains':
                    print("Sagittal kp: " + str(xrlo.get_sagittal_kp_gains_all()))
                    print("Sagittal kd: " + str(xrlo.get_sagittal_kd_gains_all()))
                    print("Frontal kp: " + str(xrlo.get_frontal_kp_gains_all()))
                    print("Frontal kd: " + str(xrlo.get_frontal_kd_gains_all()))
                elif i=='print pos':
                    print("Positions: " + str(xrlo.get_pos_all()))

        elif(state =='thts'):
            while True:
                i = input("Press Enter to return, q+Enter to quit, or one of the following to configure:\ngo home\nread thts minus offsets\ndes thts minus offsets\nread thts\ndes thts\noffsets\nsend thts\nset home...")
                if not i:
                    state = 'configure'
                    break
                if i=='q':
                    cleanQuit()
                elif i=='read thts minus offsets':
                    pos_minus_offsets = xrlo.read_thts()
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            pos_minus_offsets[i][j][0] = pos_minus_offsets[i][j][0] - offsets[i][j][0]
                            pos_minus_offsets[i][j][1] = pos_minus_offsets[i][j][1] - offsets[i][j][1]
                            #htDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j
                    print("Positions minus offsets: " + str(pos_minus_offsets))
                elif i=='des thts minus offsets':
                    pos_minus_offsets = xrlo.read_thts()
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            pos_minus_offsets[i][j][0] = thtDesired[i][j][0] - offsets[i][j][0]
                            pos_minus_offsets[i][j][1] = thtDesired[i][j][1] - offsets[i][j][1]
                            #htDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j
                    print("Desired position minus offsets: " + str(pos_minus_offsets))
                elif i=='des thts':
                    print(thtDesired)
                elif i=='read thts':
                    print(xrlo.read_thts())
                elif i=='offsets':
                    print(offsets)
                elif i=='go home':
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                            thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
                    commAll()
                elif i=='send thts':
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                            thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
                    commAll()
                elif i=='set home':
                    home_thts = xrlo.read_thts()
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            home_thts[i][j][0] -= squat_home[i][j]
                    offsets = home_thts
                    print(offsets)
                    print("To permanently save this home position, copy the above array and paste into xrl_odrive_py as offsets.")
                    for i in range(0,len(odrvs)):
                        for j in range(0,len(odrvs[0])):
                            thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                            thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
                    commAll()

        elif(state =='gains'):
            while True:
                i = input("Press Enter to return, q+Enter to quit, or one of the following to configure:\nprint\nall\nmax all\nsingle joint\njoint pair...")
                if not i:
                    state = 'configure'
                    break
                if i=='q':
                    cleanQuit()
                elif i=='print':
                    print("Sagittal kp: " + str(xrlo.get_sagittal_kp_gains_all()))
                    print("Sagittal kd: " + str(xrlo.get_sagittal_kd_gains_all()))
                    print("Frontal kp: " + str(xrlo.get_frontal_kp_gains_all()))
                    print("Frontal kd: " + str(xrlo.get_frontal_kd_gains_all()))
                elif i=='max all':
                    xrlo.max_gains_all()
                elif i=='single joint':
                    #get parameters
                    leg = get_int_num_from_user('leg', range(2))
                    joint = get_int_num_from_user('joint', range(3))
                    #gear = get_bool_from_user('geared')
                    s_kp = get_float_num_from_user('sagittal kp', 0.0, 5000.0)
                    s_kd = get_float_num_from_user('sagittal kd', 0.0, 20.0)
                    f_kp = get_float_num_from_user('frontal kp', 0.0, 5000.0)
                    f_kd = get_float_num_from_user('frontal kd', 0.0, 20.0)
                    xrlo.ramp_up_gains(leg, joint, s_kp, s_kd, f_kp, f_kd, rampSec=5, hz=100, debug=False)
                elif i=='joint pair':
                    #get parameters
                    joint = get_int_num_from_user('joint', range(3))
                    #gear = get_bool_from_user('geared')
                    s_kp = get_float_num_from_user('sagittal kp', 0.0, 5000.0)
                    s_kd = get_float_num_from_user('sagittal kd', 0.0, 20.0)
                    f_kp = get_float_num_from_user('frontal kp', 0.0, 5000.0)
                    f_kd = get_float_num_from_user('frontal kd', 0.0, 20.0)
                    xrlo.ramp_up_gains(0, joint, s_kp, s_kd, f_kp, f_kd, rampSec=5, hz=100, debug=False)
                    xrlo.ramp_up_gains(1, joint, s_kp, s_kd, f_kp, f_kd, rampSec=5, hz=100, debug=False)
                elif i=='all':
                    #gear = get_bool_from_user('geared')
                    #get parameters
                    s_kp = get_float_num_from_user('sagittal kp', 0.0, 5000.0)
                    s_kd = get_float_num_from_user('sagittal kd', 0.0, 20.0)
                    f_kp = get_float_num_from_user('frontal kp', 0.0, 5000.0)
                    f_kd = get_float_num_from_user('frontal kd', 0.0, 20.0)

                    xrlo.ramp_up_gains_all_sagittal(s_kp, s_kd, rampSec=5, hz=100, debug=False)
                    xrlo.ramp_up_gains_all_frontal(f_kp,f_kd, rampSec=5, hz=100, debug=False)
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
        elif(state == 'dynamic gains'):
            print("NOT IMPLEMENTED")
            print("RETURNING TO CONFIGURE")
            state = 'configure'

            '''
            rampTime = seconds
            dynamic_gains = True

            #need some sort of input for the xrlk functions, like squat has yDes
            kPVals = home_kp #xrlk. nonexistent dynamic gains function!
            kDVals = home_kd #xrlk. nonexistent dynamic gains function!
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    #sagittal kp
                    kPDesired[i][j][0] = kPVals[leg][joint][0]
                    #sagittal kd
                    kDDesired[i][j][0] = kDVals[leg][joint][0]
                    #frontal kp
                    kPDesired[i][j][1] = kPVals[leg][joint][1]
                    #frontal kd
                    kDDesired[i][j][1] = kDVals[leg][joint][1]

            myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))
            #print('Des:',niceList(thtDesired))
            #if reach low position, stand back up
            if(t - tStartSquat >= rampTime):
                tStartUp = t
                state = 'standup'
                print('-------------------------------State: ',state)
            '''

        elif(state == 'waitforsquat'):
            for i in range(0,len(odrvs)):
                for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
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
                wait_down = get_bool_from_user('wait at bottom')
                seconds = get_int_num_from_user('seconds', range(1000))
                height = get_float_num_from_user('height to squat to (max 53.7, min 33) in inches', 32.75, max_height)
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
            rampTime = seconds
            xDes = 0
            #For real robot: No lower than 32.75, midpoint is 42, max is 53.70
            if(t-tStartSquat < rampTime):
                yDes = (max_height-(((t - tStartSquat)/rampTime))*(max_height - height))*in2m
                thtVals = xrlk.FrontalIK(xDes,yDes)
                for i in range(0,len(odrvs)):
                    for j in range(0,len(odrvs[0])):
                        thtDesired[i][j][0] = thtVals[i][j]+offsets[i][j][0]
                        thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
                myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))
            #print('Des:',niceList(thtDesired))
            #if reach low position, stand back up
            if(t - tStartSquat>= rampTime):
                tStartUp = t
                state = 'standup'
                print('-------------------------------State: ',state)
        elif(state == 'standup'): # stand up over period of time
            #IK, log all feedback over time
            while wait_down:
                print("waiting")
                i = input("Press Enter to continue, or q+Enter to quit...")
                if not i:
                    wait_down = False
                    tStartUp = time.time() - tStart
                    t = time.time() - tStart
                    print("done waiting!")
                if i=='q':
                    cleanQuit()

            rampTime = seconds
            #print(rampTime)
            #print(tStartUp)
            #print(t)
            xDes = 0
            #For real robot: No lower than 32.75, midpoint is 42, max is 53.70
            if(t-tStartUp < rampTime):
                yDes = (max_height-(1-((t - tStartUp)/rampTime))*(max_height - height))*in2m
                print(yDes)
                thtVals = xrlk.FrontalIK(xDes,yDes) # returns values that are then put in sagittal?
                print("theta vals")
                print(thtVals)
                #print(thtVals)
                for i in range(0,len(odrvs)):
                    for j in range(0,len(odrvs[0])):
                        thtDesired[i][j][0] = thtVals[i][j]+offsets[i][j][0]
                        thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
                myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))
            #print('Des:',niceList(thtDesired))
            #If reach top, idle
            if(t - tStartUp>= rampTime):
                for i in range(0,len(odrvs)):
                    for j in range(0,len(odrvs[0])):
                        thtDesired[i][j][0] = squat_home[i][j]+offsets[i][j][0]
                        thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
                myLogger.appendData(str([t,thtDesired,thtActual,curCommand]))
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
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand, offsets
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
    print('Time of End: ',t)
    isRunning = False
    print('-----------------Quitting...')
    print('-----------------Ramping Gains Down...')
    #deal with gains
    dynamic_gains = False
    xrlo.ramp_up_gains_all_sagittal(0, 0.0, rampSec=1) #use cpr2rad if NOT in mixed mode
    xrlo.ramp_up_gains_all_frontal(0 ,0.0, rampSec=1)
    print('-----------------All Gains at low')

    #deal with odrive axis states
    for leg in range(0,len(odrvs)):
        for joint in range(0,len(odrvs[0])):
            if odrvs[leg][joint] == None:
                continue
            odrvs[leg][joint].axis0.requested_state = AXIS_STATE_IDLE
            odrvs[leg][joint].axis1.requested_state = AXIS_STATE_IDLE
    print('-----------------All Motors idle')

    '''
    #thtVals, thtDesired always get reset whenever we boot up
    #so this isn't necessary
    thtVals = xrlk.FrontalIK(0,53.7*in2m)
    for i in range(0,len(odrvs)):
        for j in range(0,len(odrvs[0])):
                    thtDesired[i][j][0] = -thtVals[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = 0+offsets[i][j][1] #+ thtVals[i][j]
    '''
    commAll()
    print('-----------------Quit!')
    sys.exit(0)

def odrv_comm(leg, joint):
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand, offsets
    ### Send Commands

    ###Mixed Position Control
    #both, sagittal, frontal
    ### DEBUGGING
    print("leg: " + str(leg) + ", joint: " + str(joint) + ", thtDesired_s: " + str(thtDesired[leg][joint][0]) + ", thtActual_s: " + str(odrvs[leg][joint].axis0.controller.theta_s)+ ", thtDesired_f: " + str(thtDesired[leg][joint][1]) + ", thtActual_f: " + str(odrvs[leg][joint].axis0.controller.theta_f))
    #odrvs[leg][joint].axis0.controller.set_mixed_pos_setpoint(True, thtDesired[leg][joint][0], 0)

    ###Mixed Pos&Vel Control?
    #both, sagittal, frontal, sagittal_vel, frontal_vel
    odrvs[leg][joint].axis0.controller.set_mixed_setpoint(True, thtDesired[leg][joint][0], thtDesired[leg][joint][1], 0, 0) #velDesired[leg][joint][0] as second to last term
    #last argument should eventually be velDesired[leg][joint][1]

    ###Mixed Gains
    ###this currently is not working
    ###whole program is currently in a single set gain mode
    #no ability to change the gains in real time
    if False:#dynamic_gains:
        odrvs[leg][joint].axis0.controller.set_mixed_gains(True, kPDesired[leg][joint][0], kPDesired[leg][joint][1], kDDesired[leg][joint][0], kDDesired[leg][joint][1])



    ### Read Current States
    #position
    thtActual[leg][joint][0] = odrvs[leg][joint].axis0.controller.theta_s
    thtActual[leg][joint][1] = odrvs[leg][joint].axis0.controller.theta_f
    #thtActual[leg][joint][0] = odrvs[leg][joint].axis0.encoder.pos_estimate * CPR2RAD
    #thtActual[leg][joint][1] = odrvs[leg][joint].axis1.encoder.pos_estimate * CPR2RAD
    #velocity
    velActual[leg][joint][0] = odrvs[leg][joint].axis0.encoder.pll_vel
    velActual[leg][joint][1] = odrvs[leg][joint].axis1.encoder.pll_vel
    #current
    curCommand[leg][joint][0] = odrvs[leg][joint].axis0.motor.current_control.Iq_measured
    curCommand[leg][joint][1] = odrvs[leg][joint].axis1.motor.current_control.Iq_measured

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
