"""
XRL Serial Drive Code
dgonz@mit.edu
"""
import serial
import struct
import signal
import sys
import time
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

thtActual = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
velActual =   [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
curCommand = [[[0,0],[0,0],[0,0]],[[0,0], [0,0], [0,0]]]
myLogger = dataLogger('data.txt')

BAUD = 921600
#Robot facing 3DP:
#                   RIGHT LEG                    LEFT LEG
#ports = [['COM36', 'COM35', 'COM34'],['COM37', 'COM31', 'COM32']]
#ports = [['COM37', 'COM31', 'COM32']]
#ports = [['COM35', 'COM34'],['COM31','COM32']]  #for debug
ports = [['COM36']]

ss = [[None, None, None],[None, None, None]]

for i in range(0,len(ports)):
    for j in range(0,len(ports[0])):
        ss[i][j] = serial.Serial(ports[i][j],BAUD, timeout = 0.050) #15ms timeout
        ss[i][j].flush()

isRunning = True
isAvailable = False
t = 0

def main():
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand, t
    myLogger.appendData('\n--NewTrial--\n')
    titleStr = 't,'
    for i in range(0,len(ports)):
        for j in range(0,len(ports[0])):
            for k in range(0,2):
                titleStr+='thtDesired'+str(i)+str(j)+str(k)+','
    for i in range(0,len(ports)):
        for j in range(0,len(ports[0])):
            for k in range(0,2):
                titleStr+='thtActual'+str(i)+str(j)+str(k)+','
    for i in range(0,len(ports)):
        for j in range(0,len(ports[0])):
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
            for i in range(0,len(ports)):
                for j in range(0,len(ports[0])):
                    thtDesired[i][j][0] = -thtVals[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = thtVals[i][j]+offsets[i][j][1]
            # ramp up kD and kP to high for some time
            for i in range(0,len(ports)):
                for j in range(0,len(ports[0])):
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
            for i in range(0,len(ports)):
                for j in range(0,len(ports[0])):
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
            for i in range(0,len(ports)):
                for j in range(0,len(ports[0])):
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
    for i in range(0,len(ports)):
        for j in range(0,len(ports[0])):
            doComm2(i, j)
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
        for i in range(0,len(ports)):
            for j in range(0,len(ports[0])):
                ss[i][j].flush()
                for k in range(0,2):
                    kD[i][j][k] = 0.1+(1-(t/rampTime))*(kDCurr[i][j][k]-0.1)
                    kP[i][j][k] = 1+(1-(t/rampTime))*(kPCurr[i][j][k]-1)
        commAll()
    print('-----------------All Gains at low')
    thtVals = xrlk.FrontalIK(0,53.7*in2m)
    for i in range(0,len(ports)):
        for j in range(0,len(ports[0])):
                    thtDesired[i][j][0] = -thtVals[i][j]+offsets[i][j][0]
                    thtDesired[i][j][1] = thtVals[i][j]+offsets[i][j][1]
#    kP = zeroVec
#    kD = zeroVec
    for i in range(0,len(ports)):
        for j in range(0,len(ports[0])):
            doComm2(i, j)
            ss[i][j].close()
    #ss[0][0].close() #for debug
    print('-----------------Quit!')
    sys.exit(0)

def doComm(leg, joint):
    global thtDesired, velDesired, kP, kD, thtActual, velActual, curCommand
    #print('Leg '+str(leg)+' Joint '+str(joint)+':')
    ss[leg][joint].flush()
    msg = ss[leg][joint].read(1)
    #print(msg)  #for debug
    if(msg!=b'S' or msg!=b'T'):
        if(msg==b'b'):
            msg = ss[leg][joint].readline()
            print(str(msg))
        msg = ss[leg][joint].read(1)
    #print(msg) #for debug
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
        if(msg[-1:] == b'\n'):
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
            #print(msgOut, len(msgOut))  #for debug
            ss[leg][joint].write(msgOut)
            #print('LMAO') #for debug
    else:
        print('From ',str(leg),str(joint),'Not S: ',msg,'\n', len(msg))

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


signal.signal(signal.SIGINT, cleanQuitInt)
main()
