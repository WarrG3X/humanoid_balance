#!/usr/bin/env python
import rospy
import time
import pypot.dynamixel
from math import pi,acos,tan,log,degrees,radians
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from humanoid_balance.msg import botinfo
from pynamixel.msg import Actuation


#Constants
b = 0.0925       #Shin Link
c = 0.0925       #Thigh Link
h = b+c          #Leg Height
d = 0.064        #Footplate Distance

#Parameters
SPEED = 1023
LOCK = 19
AXIS1 = 'x'
AXIS2 = 'y'
AXIS3 = 'z'
FLAG = None

#Variables
dxl_io = None
ids = [11,12,13,14,15,16,17,18]
angles = {x:0 for x in ids}

def initialize_dxl():
    global dxl_io
    global ids
    port_id = 0
    ports = pypot.dynamixel.get_available_ports()
    if not ports:
        raise IOError('no port found!')

    print('ports found', ports)
    print('connecting on the first available port:', ports[port_id])
    dxl_io = pypot.dynamixel.DxlIO(ports[port_id])
    ids = dxl_io.scan(range(25))
    speeds = [SPEED for x in ids]

    if len(ids) != LOCK:
        print ids
        raise RuntimeError("Couldn't detect all motors.")

    dxl_io.set_moving_speed(dict(zip(ids,speeds)))


def initialize_robot():
    raw_input("Initialize?")
    init_angles = [0 for x in ids]
    dxl_io.set_goal_position(dict(zip(ids,init_angles)))


def get_angles(t):
    angleA = degrees(acos((b**2 + c**2 + 2*h*d*tan(t) -h**2-(d**2)*(tan(t)**2))/(2*b*c)))-180
    angleB = degrees(acos((c**2 + h**2 +(d**2)*(tan(t)**2) -b**2-2*h*d*tan(t))/(2*c*(h-d*tan(t)))))
    angleC = -degrees(acos((b**2 + h**2 +(d**2)*(tan(t)**2) -c**2-2*h*d*tan(t))/(2*b*(h-d*tan(t)))))
    return angleA,angleB,angleC

def publish_pos(angles):
    msg = Actuation()
    msg.ids = range(11,19)
    msg.speeds = [1023 for id in ids]
    msg.angles = angles
    pubpos.publish(msg)

def publish_log(angles,yaw,pitch,roll):
    message = botinfo()
    message.yaw = yaw
    message.pitch = pitch
    message.roll = roll
    message.motor11 = angles[11]
    message.motor12 = angles[12]
    message.motor13 = angles[13]
    message.motor14 = angles[14]
    message.motor15 = angles[15]
    message.motor16 = angles[16]
    message.motor17 = angles[17]
    message.motor18 = angles[18]

    pub.publish(message)

def leftleg(t,debug=True):
    global angles
    print("Left")
    if debug:
        return
    theta = degrees(t)
    if abs(theta) > 10:
        #factor = log(abs(theta),10)
        factor = 1
    else:
        factor = 1


    angleA,angleB,angleC = get_angles(t)
    angles ={11:angles[11],12:angleB,13:angles[13],14:angleA,15:angles[15],16:angleC,17:factor*theta,18:factor*theta}

def rightleg(t,debug=True):
    global angles
    print("right")
    if debug:
        return
    theta = degrees(t)
    if abs(theta) > 10:
        #factor = log(abs(theta),10)
        factor = 1
    else:
        factor = 1

    angleA,angleB,angleC = get_angles(abs(t))
    angles = {11:-angleB,12:angles[12],13:-angleA,14:angles[14],15:-angleC,16:angles[16],17:factor*theta,18:factor*theta}

def sagittal_balance(phi):
    #Phi is in Degrees
    global angles
    if abs(phi) > 10:
        #factor = log(abs(phi),10)
        factor = 1
    else:
        factor = 1

    if FLAG == 'LEFT':
        angles[15] = -factor*phi
        angles[16] += factor*phi
    elif FLAG == 'RIGHT':
        angles[15] += -factor*phi
        angles[16] = factor*phi
    else:
        angles[15] += -factor*phi
        angles[16] += factor*phi


def get_rpy(data):
    global angles
    global FLAG
    t = getattr(data.vector,AXIS1)
    p = getattr(data.vector,AXIS2)
    s = getattr(data.vector,AXIS3)
    sign = t/abs(t)
    t = sign*abs(pi-abs(t))
    theta = degrees(t)
    phi = degrees(p)
    psi = degrees(s)
    #print phi

    if theta > 0 and theta < 25:
        leftleg(t,debug=True)
        FLAG = 'LEFT'
        pass
    elif theta < 0 and theta > -25:
        rightleg(t,debug=True)
        FLAG = 'RIGHT'
        pass
    else:
        angles = {x:0 for x in ids}
        print "Not in Range"
        FLAG = None
        pass
    
    if phi < 50 and phi >-50:
        sagittal_balance(phi)
        pass

    print angles
    publish_pos(angles)
    publish_log(angles,psi,phi,theta)
    #dxl_io.set_goal_position(angles)

if __name__ == '__main__':
    rospy.init_node('balance',anonymous=False)
    #initialize_dxl()
    #initialize_robot()
    raw_input("Begin?")
    pub = rospy.Publisher('robotlog',botinfo,queue_size=10)
    pubpos = rospy.Publisher('actuation',Actuation,queue_size=10)
    rospy.Subscriber("/imu/rpy/filtered",Vector3Stamped,get_rpy)
    rospy.spin()
