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
AXIS = 'x'

#Variables
dxl_io = None
ids = [11,12,13,14,15,16,17,18]

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
    print("Left")
    if debug:
        return
    theta = degrees(t)
    angleA,angleB,angleC = get_angles(t)
    angles = {17:theta,18:theta}
    dxl_io.set_goal_position(angles)
    time.sleep(0.01)
    angles = {11:0,12:angleB,13:0,14:angleA,15:0,16:angleC}
    dxl_io.set_goal_position(angles)

def rightleg(t,debug=True):
    print("right")
    if debug:
        return
    theta = degrees(t)
    angleA,angleB,angleC = get_angles(abs(t))
    angles = {17:theta,18:theta}
    dxl_io.set_goal_position(angles)
    time.sleep(0.01)
    angles = {11:-angleB,12:0,13:-angleA,14:0,15:-angleC,16:0}
    dxl_io.set_goal_position(angles)



def get_rpy(data):
    t = getattr(data.vector,AXIS)
    sign = t/abs(t)
    t = sign*abs(pi-abs(t))
    theta = degrees(t)
    print theta
    if theta > 0.1 and theta < 20:
        leftleg(t,debug=False)
        pass
    elif theta < -0.1 and theta > -20:
        rightleg(t,debug=False)
        pass
    else:
        print "Not in Range"
        pass
    





if __name__ == '__main__':
    rospy.init_node('humanoid_lateral_balance',anonymous=False)
    initialize_dxl()
    initialize_robot()
    raw_input("Begin?")
    rospy.Subscriber("/imu/rpy/filtered",Vector3Stamped,get_rpy)
    rospy.spin()
