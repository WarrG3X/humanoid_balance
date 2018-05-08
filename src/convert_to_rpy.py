import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Vector3


def convert_to_rpy(imu_data):
    quaternion = imu_data.orientation
    quaternion_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
    rpy = Vector3()
    rpy.x = roll
    rpy.y = pitch
    rpy.z = yaw
    pub_rpy.publish(rpy)
    
    
def start():
    rospy.init_node('rpy_generator', anonymous=True)
    rospy.Subscriber("/rtimulib_node/imu", Imu, convert_to _rpy)
    rospy.spin()

if __name__ == '__main__':
    pub_rpy = rospy.Publisher('rpy', Vector3, queue_size=10)
    start()
