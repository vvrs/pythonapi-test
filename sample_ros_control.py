#!/usr/bin/env python

import rospy
from ackermann_msgs.msg import AckermannDrive

def talker():
    pub = rospy.Publisher('/carla/ego_vehicle/ackermann_cmd', AckermannDrive, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        
        ackermann_cmd = AckermannDrive()
        ackermann_cmd.steering_angle = 0.2
        ackermann_cmd.speed = 0.0
        pub.publish(ackermann_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass