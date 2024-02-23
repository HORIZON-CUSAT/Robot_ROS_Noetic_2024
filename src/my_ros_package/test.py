#!/usr/bin/env python

import rospy
from std_msgs.msg import String


dx =0.0
dy=0.0

prev_time=0
prev=rospy.Time.from_sec(prev_time)

def rpm_callback(data):
    # Split the received string by ',' to get individual RPM values
    rpm_values = data.data.split(',')
    current_time = rospy.Time.now()
    
    if len(rpm_values) == 4:
	global prev
	global dx,dy
        # Print RPM values of four motors
        motor1_rpm = rpm_values[0]
        motor2_rpm = rpm_values[1]
        motor3_rpm = rpm_values[2]
        motor4_rpm = rpm_values[3]
        
        print("Motor 1 RPM:",motor1_rpm)
        print("Motor 2 RPM:",motor2_rpm)
        print("Motor 3 RPM:",motor3_rpm)
        print("Motor 4 RPM:",motor4_rpm)

	right_rpm=(float(motor4_rpm)+float(motor2_rpm))/2
	left_rpm=(float(motor1_rpm)+float(motor3_rpm))/2
        delta=prev-current_time
	wheelbase=0.45
	
	time1=float(delta.to_sec())
	print("int",time1)
	#distance=(-time1)*float(motor1_rpm)*2*3.14*10/100+distance
	dx=(-time1)*float((right_rpm+left_rpm)/2)*2*3.14*10/6000+dx
	dy=(-time1)*float((right_rpm-left_rpm)/2)*2*3.14*10/6000*wheelbase+dy
	prev=current_time
	print("dx",dx)
	print("dy",dy)

    else:
        rospy.logwarn("Received an invalid message format. Expected 4 RPM values separated by ','.")

def rpm_listener():
    rospy.init_node('rpm_listener', anonymous=True)
    
    # Subscribe to the "rpm_wheels" topic with String type messages
    rospy.Subscriber('rpm_wheels', String, rpm_callback)
    
    rospy.spin()

try:
    rpm_listener()
except rospy.ROSInterruptException:
    pass

