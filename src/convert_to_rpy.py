import rospy
from sensor_msgs.msg import Imu

def convert_to_rpy(imu_data):
    quaternion = imu_data.orientation
    print quaternion
    
def listener():
    rospy.init_node('rpy_generator', anonymous=True)
    rospy.Subscriber("/rtimulib_node/imu", Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    start()
