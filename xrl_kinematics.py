"""
XRL Kinematics
dgonz@mit.edu
"""

import numpy as np
from numpy import cos, sin, pi, arctan2, arccos, zeros
from numpy.linalg import norm

##  Constants
in2mm = 25.4
mm2in = 1/in2mm
in2m = in2mm/1000
lbf2N = 4.448
N2lbf = 1/lbf2N
kg2lbs = 2.205
lbs2kg = 1/kg2lbs
phi = 1.618 #Golden Ratio
g = 9.806 #Acceleration due to Earth's Gravity
RAD2DEG = (360.0/(2.0*pi))
DEG2RAD = 1.0/RAD2DEG


##  Important Geometric Design Parameters
h_person = 68*in2m #Height of person, in inches. Right now, I'm 5ft 8in
#2.5% to 97.5: 64.5*in2m; 76.5*in2m;
l_h2r_hip = 10*in2m #x-Distance from person's COM to robot's COM, 13.4375, 10.25
h_stair = 0*in2m #Normally 8 inches, adds to leg length
w_hip = 30.5*in2m #Design Parameter

##  Derived or Unimportant Parameters
l_hip = 0#6*in2m #Distance from Robot+pack COM to Robot's hip in z direction ----------------------------IMPORTANT, CHANGES THE HEIGHT! EEEEEEEEP
l_thigh = (20/68)*h_person #Height from ground to person's hip during crawling
h_crawl = l_thigh+l_h2r_hip
h_hip = h_person*0.75
l_total = h_hip+h_stair
#l_1 = (l_total-h_crawl)/2
l_1 = 10.5*in2m
#l_2 = l_total - l_1
l_2 = 40.5*in2m
l_3 = (10.5-2.5)*in2m
w_3 = 6*in2m #Width of foot
h_3 = 2.75*in2m #height of foot
l_bottom = l_2-l_1-l_hip+.125*in2m #13.5

totalLength_Inches = l_total/in2m
link1_Inches = l_1/in2m
link2_Inches = l_2/in2m

##  Force Parameters
m_bot = 80*lbs2kg #35 [kg] Assume massless legs, all mass at COM
m_payload = 50*lbs2kg #25 [kg]

F_assist = 1*50*lbf2N
Fy_ext = 0; #[N]
Fz_ext = (-(m_bot+m_payload)*g - F_assist) #[N]
Tx_ext = 0 #[Nm]

#   Simulation Parameters
opts = [0, w_3, w_hip, 36*in2m, w_hip+2*l_1, 48*in2m]
w_base = 27.25*in2m #Width of foot base.

if (F_assist<=0):
    w_base = opts[2]

z_r = np.arange(l_bottom,h_hip-l_hip-0.01,0.01)

if (F_assist<=0):
    y_r = np.arange(0, w_3, 0.01)
else:
    y_r = np.arange(0, w_base/2+w_3/2, 0.01)

tht_r = 0
x_e = y_r
y_e = z_r
tht_x = tht_r

# Note that q1,2,3 are hip, knee, and ankle respectively. 
# q = [0, 0, 0] is standing straight up
# Returns 3x3 Jacobian for the planar Sagittal View
def JacSag(q =  np.array([0, 0, 0]), l_y = l_hip, l_x = 0):
    return np.array(\
        [[-l_x*cos(q[0]+q[1]+q[2])-l_y*sin(q[0]+q[1]+q[2]+pi/2-pi),\
            -l_x*cos(q[0]+q[1]+q[2])-l_y*sin(q[0]+q[1]+q[2]+pi/2-pi) - l_1*sin(q[2]+q[1]+pi/2),\
                -l_x*cos(q[0]+q[1]+q[2])-l_y*sin(q[0]+q[1]+q[2]+pi/2-pi) - l_1*sin(q[2]+q[1]+pi/2) - l_2*sin(q[2]+pi/2)],\
        [l_x*sin(q[0]+q[1]+q[2])+l_y*cos(q[0]+q[1]+q[2]+pi/2-pi),\
            l_x*sin(q[0]+q[1]+q[2])+l_y*cos(q[0]+q[1]+q[2]+pi/2-pi) + l_1*cos(q[2]+q[1]+pi/2),\
                l_x*sin(q[0]+q[1]+q[2])+l_y*cos(q[0]+q[1]+q[2]+pi/2-pi) + l_1*cos(q[2]+q[1]+pi/2) + l_2*cos(q[2]+pi/2)],\
        [1, 1, 1]])

# Robot Facing us, the Robot's Left (On our Right) leg values are as follows when squatting down
# Ankle: negative, Knee: Positive, Ankle: negative
# q = [rightHip, rightKnee, rightAnkle, leftHip, leftKnee, leftAnkle]
# Returns 3x6 Jacobian for the planar Frontal View
def JacFront(q = np.array([0, 0, 0, 0, 0, 0])):
    Js1 = JacSag(q[0:3],0, w_hip/2)
    Js2 = JacSag(q[3:6],0, w_hip/2)
    return np.concatenate((Js1,Js2), axis=1)/2

def FrontalIK(x,y,tht_x = 0):
    tht3L = 0
    tht2L = 0
    tht1L = 90*DEG2RAD
    tht3R = 0
    tht2R = 0
    tht1R = 90*DEG2RAD
    #   Body Out IK, note that coords are temorarily in traditional "X and Y"
    x_hipL = x + w_hip/2*cos(tht_x) - l_hip*sin(tht_x)
    y_hipL = y + w_hip/2*sin(tht_x) + l_hip*cos(tht_x)-h_3
    x_hipR = x + w_hip/2*cos(tht_x+pi) + l_hip*sin(tht_x)
    y_hipR = y + w_hip/2*sin(tht_x+pi) + l_hip*cos(tht_x)-h_3

    #   Solve for left and right leg
    #print(norm([x_hipL-w_base/2, y_hipL]))
    #print((x_hipL-w_base/2,y_hipL))
    #print(norm([x_hipR+w_base/2, y_hipR]))
    #print((x_hipR+w_base/2,y_hipR))
    if(norm([x_hipL-w_base/2, y_hipL])<l_1+l_2 and \
       norm([x_hipL-w_base/2, y_hipL])>l_2-l_1):
        #If this is a valid point
        (tht1L, tht2L) = TwoLinkIK(x_hipL-w_base/2,y_hipL, l_2, l_1, 1)
    if(norm([x_hipR+w_base/2, y_hipR])<l_1+l_2 and \
       norm([x_hipR+w_base/2, y_hipR])>l_2-l_1):
        #If this is a valid point
        (tht1R, tht2R) = TwoLinkIK(x_hipR+w_base/2,y_hipR, l_2, l_1, 0)
    """
    tht3L = pi/2-tht1L #ankle
    tht2L = -tht2L
    tht1L = (tht1L-tht2L) - pi/2-tht_x
    tht3R = tht1R-pi/2 #ankle
    tht2R = tht2R
    tht1R = -(tht1R+tht2R) + pi/2+tht_x
    """
    tht3L = pi/2-tht1L #ankle
    tht2L = -tht2L
    tht1L = (tht1L-tht2L) - pi/2 - tht_x
    tht3R = tht1R-pi/2 #ankle
    tht2R = tht2R
    tht1R = -(tht1R+tht2R) + pi/2 + tht_x
    #return(((-tht3L)*RAD2DEG, tht2L*RAD2DEG, (tht1L-pi/2)*RAD2DEG),\
    #       ((-tht3R)*RAD2DEG, tht2R*RAD2DEG, (tht1R-pi/2)*RAD2DEG))
    return(((tht1L), tht2L, (tht3L)),\
           ((tht1R), tht2R, (tht3R)))

def TwoLinkIK(x, y, l_1, l_2, opt = 0):
    alpha = np.arctan2(y,x)
    theta_2 = np.pi-np.arccos((l_1*l_1+l_2*l_2-x*x-y*y)/(2*l_1*l_2))
    gamma = np.arccos((x*x+y*y+l_1*l_1-l_2*l_2)/(2*l_1*np.sqrt(x*x+y*y)))
    theta_1 = alpha-gamma
    if(opt):
      theta_1 = theta_1+2*gamma
      theta_2 = -theta_2
    return (theta_1, theta_2)

def test():
    global w_base, w_hip
    #print(totalLength_Inches)
    print((l_1+l_2+h_3)/in2m)
    print((l_2-l_1+h_3)/in2m)
    #w_base = 22*in2m
    #w_hip  = 22*in2m

    #For real robot: No lower than 32.75, midpoint is 42, max is 53.70
    for i in np.arange(32.75,53.8,0.05):
        print('Height of ',i, ' inches: ',niceListKin(FrontalIK(0,i*in2m)))

def niceListKin(theList):
    return ["%0.2f" % i for i in flattenListKin(theList)]

def flattenListKin(container):
    for i in container:
        if isinstance(i, (list,tuple)):
            for j in flattenListKin(i):
                yield j
        else:
            yield i
