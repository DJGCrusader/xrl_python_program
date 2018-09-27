"""
Multi Wheatstone Bridge Interface for DOE XRL Harness
Daniel J. Gonzalez and Maria Rosa Ruiz
dgonz@mit.edu
Spring 2017
"""

#Basic imports
import sys
from time import sleep, time
#Phidget specific imports
from Phidget22.PhidgetException import PhidgetException
from Phidget22.Devices.Bridge import Bridge, BridgeGain
from Phidget22.Phidget import PhidgetLogLevel

rawVals = [0, 0, 0, 0, 0, 0]
calibOffset = [0.1332, .09933, .3908, .1442, 0.0354, 0.1488]

#Information Display Function
def displayDeviceInfo(bridge):
    print("|------------|----------------------------------|--------------|------------|")
    print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
    print("|------------|----------------------------------|--------------|------------|")
    print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (bridge.isAttached(), bridge.getDeviceName(), bridge.getSerialNum(), bridge.getDeviceVersion()))
    print("|------------|----------------------------------|--------------|------------|")
    print("Number of bridge inputs: %i" % (bridge.getInputCount()))
    print("Data Rate Max: %d" % (bridge.getDataRateMax()))
    print("Data Rate Min: %d" % (bridge.getDataRateMin()))
    print("Input Value Max: %d" % (bridge.getBridgeMax(0)))
    print("Input Value Min: %d" % (bridge.getBridgeMin(0)))

#Event Handler Callback Functions
def BridgeError(e):
    try:
        source = e.device
        print("Bridge %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))

def BridgeData0(e):
    global rawVals
    rawVals[e.index] = e.value
    
def BridgeData1(e):
    global rawVals
    rawVals[4+e.index] = e.value

def initialize():
    global bridge0, bridge1
    #Create two Bridge objects
    try:
        bridgeA = Bridge()
        bridgeB = Bridge()
    except RuntimeError as e:
        print("Runtime Exception: %s" % e.details)
        print("Exiting....")
        exit(1)
    print("Opening phidget objects....")
    bridgeA.openPhidget()
    bridgeB.openPhidget()
    print("Waiting for attach....")
    try:
        bridgeA.waitForAttach(10000)
        bridgeB.waitForAttach(10000)
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))
        try:
            bridgeA.closePhidget()
            bridgeB.closePhidget()
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(1)
        print("Exiting....")
        exit(1)
    else:
        displayDeviceInfo(bridgeA)
        displayDeviceInfo(bridgeB)
    if bridgeA.getSerialNum()==405266:
        bridge0=bridgeA
        bridge1=bridgeB
    else:
        bridge0=bridgeB
        bridge1=bridgeA
    print(bridge0.getSerialNum(), bridge1.getSerialNum())
    sleep(3)
    bridge0.setOnBridgeDataHandler(BridgeData0)
    bridge1.setOnBridgeDataHandler(BridgeData1)
    print("Set data rate to 8ms ...")
    bridge0.setDataRate(16)
    bridge1.setDataRate(16)
    print("Set Gain to 8...")
    bridge0.setGain(0, BridgeGain.PHIDGET_BRIDGE_GAIN_8)
    bridge1.setGain(0, BridgeGain.PHIDGET_BRIDGE_GAIN_8)
    print("Enable the Bridge inputs for reading data...")
    bridge0.setEnabled(0, True)
    bridge0.setEnabled(1, True)
    bridge0.setEnabled(2, True)
    bridge0.setEnabled(3, True)
    bridge1.setEnabled(0, True)
    bridge1.setEnabled(1, True)
    calibrate()

def close():
    global bridge0
    global bridge1
    print("Disable the Bridge input for reading data...")    
    bridge0.setEnabled(0, False)
    bridge0.setEnabled(1, False)
    bridge0.setEnabled(2, False)
    bridge0.setEnabled(3, False)
    bridge1.setEnabled(0, False)
    bridge1.setEnabled(1, False)
    sleep(2)

    bridge0.closePhidget()
    bridge1.closePhidget()

    print("Done.")

def calibrate():
    global rawVals, calibOffset
    print ("Beginning calibration in 3...")
    sleep(3)
    print("Calibrating... This will take 5 seconds.")
    startTime = time()
    lastTime = time()
    while time()< startTime + 5:
        calibOffset = [(i[0]+i[1])/2 for i in zip(rawVals, calibOffset)]
        

def getCalibVals():
    global rawVals,calibOffset
    return [i[0]-i[1] for i in zip(rawVals, calibOffset)]

def test():
    global rawVals,calibOffset
    initialize()
    startTime = time()
    lastTime = time()
    while time()< startTime + 30:
        if(time() - lastTime>=0.1):
            print ("calibVals: ", "%.2f "*6 % tuple(getCalibVals()))
            lastTime = time()
    close()
    exit(0)
# test()
