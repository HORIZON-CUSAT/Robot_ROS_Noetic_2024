#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def control_arm():
    rospy.init_node('arm_controller', anonymous=True)
    pub = rospy.Publisher('robotic_arm_topic', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    last_command = "0,0,0,0,0,0"  # Initialize with a default value


    command = raw_input("Enter servo commands (e.g., 90,45,0,180,90,0): ")
    
    # Check if the user entered a new value
    if command:
        last_command = command

    pub.publish(last_command)
    rate.sleep()


try:
    control_arm()
except rospy.ROSInterruptException:
    pass
