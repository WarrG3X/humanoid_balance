#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3Stamped


def convert(imu_data):
    quaternion = imu_data.orientation
    quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
    rpy = Vector3Stamped()
    rpy.header = imu_data.header
    rpy.vector.x = roll
    rpy.vector.y = pitch
    rpy.vector.z = yaw
    pub_rpy.publish(rpy)
    
    
def start():
    rospy.init_node('rpy_generator', anonymous=True)
    rospy.Subscriber("/rtimulib_node/imu", Imu, convert)
    rospy.spin()

if __name__ == '__main__':
    pub_rpy = rospy.Publisher('rpy', Vector3Stamped, queue_size=10)
    start()
