#!/usr/bin/env python
import rospy
from humanoid_balance.msg import botinfo

def talker():
    pub = rospy.Publisher('chatter', botinfo, queue_size=10)
    rospy.init_node('test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
	message = botinfo()
	message.yaw = 1.0
	message.pitch = 2.0
	message.roll = 3.0
	message.angles = [1.0 for x in range(9)]    
	rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

