#!/usr/bin/env python

import rospy
from std_msgs.msg import String

dx = 0.0
dy = 0.0

prev_time = 0
prev = rospy.Time.from_sec(prev_time)
prev_tick1 = 0
prev_tick2 = 0
prev_tick3 = 0
prev_tick4 = 0

# Define the CPR (Counts Per Revolution) for each motor
CPR1 = 500  # Example value, replace with your actual CPR
CPR2 = 500  # Example value, replace with your actual CPR
CPR3 = 500  # Example value, replace with your actual CPR
CPR4 = 500  # Example value, replace with your actual CPR

def tick_callback(data):
    # Split the received string by ',' to get individual RPM values
    tick_values = data.data.split(',')
    current_time = rospy.Time.now()

    if len(tick_values) == 4:
        global prev
        global prev_tick1, prev_tick2, prev_tick3, prev_tick4
        # Print RPM values of four motors
        motor1_tick = tick_values[0]
        motor2_tick = tick_values[1]
        motor3_tick = tick_values[2]
        motor4_tick = tick_values[3]

        print("Motor 1 ticks:", motor1_tick)
        print("Motor 2 ticks:", motor2_tick)
        print("Motor 3 ticks:", motor3_tick)
        print("Motor 4 ticks:", motor4_tick)

        delta = prev - current_time
        gain_m1 = int(motor1_tick) - prev_tick1
        gain_m2 = int(motor2_tick) - prev_tick2
        gain_m3 = int(motor3_tick) - prev_tick3
        gain_m4 = int(motor4_tick) - prev_tick4
        time1 = float(delta.to_sec())
        time1 = time1 * -1
        rpm1 = (gain_m1 / time1) * 60 / CPR1
        rpm2 = (gain_m2 / time1) * 60 / CPR2
        rpm3 = (gain_m3 / time1) * 60 / CPR3
        rpm4 = (gain_m4 / time1) * 60 / CPR4
        prev_tick1 = int(motor1_tick)
        prev_tick2 = int(motor2_tick)
        prev_tick3 = int(motor3_tick)
        prev_tick4 = int(motor4_tick)

        prev = current_time

        # Create a string containing the RPM values separated by ','
        rpm_values = "{},{},{},{}".format(rpm1, rpm2, rpm3, rpm4)

        # Publish the RPM values on the 'rpm_wheels' topic
        rpm_publisher.publish(rpm_values)

    else:
        rospy.logwarn("Received an invalid message format. Expected 4 RPM values separated by ','.")

def tick_listener():
    rospy.init_node('ticks_listener', anonymous=True)

    # Subscribe to the "pulse_counts" topic with String type messages
    rospy.Subscriber('pulse_counts', String, tick_callback)

    global rpm_publisher
    # Create a publisher for the 'rpm_wheels' topic with String type messages
    rpm_publisher = rospy.Publisher('rpm_wheels', String, queue_size=10)

    rospy.spin()

try:
    tick_listener()
except rospy.ROSInterruptException:
    pass

