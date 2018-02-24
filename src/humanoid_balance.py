#!/usr/bin/env python
import rospy
import time
import pypot.dynamixel
from math import pi,acos,tan,degrees,radians
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu


#Constants
b = 0.0925       #Shin Link
c = 0.0925       #Thigh Link
h = b+c          #Leg Height
d = 0.064        #Footplate Distance

#Parameters
SPEED = 370
LOCK = 19
AXIS1 = 'x'
AXIS2 = 'y'

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



def leftleg(t,debug=True):
    global angles
    print("Left")
    if debug:
        return
    theta = degrees(t)
    angleA,angleB,angleC = get_angles(t)
    angles ={11:0,12:angleB,13:0,14:angleA,15:angles[15],16:angleC,17:theta,18:theta}
    dxl_io.set_goal_position(angles)

def rightleg(t,debug=True):
    global angles
    print("right")
    if debug:
        return
    theta = degrees(t)
    angleA,angleB,angleC = get_angles(abs(t))
    angles = {11:-angleB,12:0,13:-angleA,14:0,15:-angleC,16:angles[16],17:theta,18:theta}
    dxl_io.set_goal_position(angles)

def sagittal_balance(phi):
    #Phi is in Degrees
    #state = dxl_io.get_present_position([15,16])
    #print state
    sagittal_angles = {15:angles[15],16:angles[16]}
    sagittal_angles[15] += -phi
    sagittal_angles[16] += phi
    #print(angles)
    #print(sagittal_angles)
    dxl_io.set_goal_position(sagittal_angles)


def get_rpy(data):
    t = getattr(data.vector,AXIS1)
    p = getattr(data.vector,AXIS2)
    sign = t/abs(t)
    t = sign*abs(pi-abs(t))
    theta = degrees(t)
    phi = degrees(p)
    print phi
    if theta > 0 and theta < 25:
        leftleg(t,debug=False)
        pass
    elif theta < 0 and theta > -25:
        rightleg(t,debug=False)
        pass
    else:
        print "Not in Range"
        pass
    
    if phi < 25 and phi >-25:
        pass
        sagittal_balance(phi)




if __name__ == '__main__':
    rospy.init_node('humanoid_lateral_balance',anonymous=False)
    initialize_dxl()
    initialize_robot()
    raw_input("Begin?")
    rospy.Subscriber("/imu/rpy/filtered",Vector3Stamped,get_rpy)
    rospy.spin()
