#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import sys, getopt
import RTIMU
import os.path
import time
import math
from geometry_msgs.msg import Vector3


def publish_rpy():
    pub_rpy = rospy.Publisher('/imu/rpy', Vector3, queue_size=10)
    rospy.init_node('rpygen', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    SETTINGS_PATH = rospy.get_param('/rpygen/config_path')
    SETTINGS_FILE = SETTINGS_PATH+"/"+rospy.get_param('/rpygen/config_file')
    rospy.loginfo("Using settings file " + SETTINGS_FILE + ".ini")
    if not os.path.exists(SETTINGS_FILE + ".ini"):
        print("Settings file does not exist, will be created")
    
    s = RTIMU.Settings(SETTINGS_FILE)
    imu = RTIMU.RTIMU(s)

    print("IMU Name: " + imu.IMUName())

    if (not imu.IMUInit()):
        print("IMU Init Failed")
        sys.exit(1)
    else:
        print("IMU Init Succeeded")

    imu.setSlerpPower(0.02)
    imu.setGyroEnable(True)
    imu.setAccelEnable(True)
    imu.setCompassEnable(True)

    poll_interval = imu.IMUGetPollInterval()
    print("Recommended Poll Interval: %dmS\n" % poll_interval)
    while not rospy.is_shutdown():
        if imu.IMURead():
            data = imu.getIMUData()
            fusionPose = list(map(math.degrees, data["fusionPose"]))
            #print("r: %f p: %f y: %f" % (math.degrees(fusionPose[0]),math.degrees(fusionPose[1]), math.degrees(fusionPose[2])))
            (roll, pitch, yaw) = fusionPose
            rpy = Vector3()
            rpy.x = roll
            rpy.y = pitch
            rpy.z = yaw
            pub_rpy.publish(rpy)
            time.sleep(poll_interval*1.0/1000.0)
            #rate.sleep()
        else:
            pass


if __name__ == '__main__':
    try:
        publish_rpy()
    except rospy.ROSInterruptException:
        pass
