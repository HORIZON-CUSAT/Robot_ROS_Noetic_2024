#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu  # Import the message type for IMU data

def imu_data_callback(data):
    # This function will be called whenever new data is received on the "imu/data" topic
    # Print the IMU data
    print("IMU Data Received:")
    print("Linear Acceleration (m/s^2):")
    #print("  x: {}".format(data.linear_acceleration.x))
    #print("  y: {}".format(data.linear_acceleration.y))
    #print("  z: {}".format(data.linear_acceleration.z))
    print(float(data.linear_acceleration.x))
    #print("Angular Velocity (rad/s):")
    #print("  x: {}".format(data.angular_velocity.x))
    #print("  y: {}".format(data.angular_velocity.y))
    #print("  z: {}".format(data.angular_velocity.z))
    #print("Orientation (Quaternion):")
    #print("  x: {}".format(data.orientation.x))
    #print("  y: {}".format(data.orientation.y))
    #print("  z: {}".format(data.orientation.z))
    #print("  w: {}".format(data.orientation.w))
    print("")

def imu_listener():
    # Initialize the ROS node
    rospy.init_node('imu_listener', anonymous=True)

    # Subscribe to the "imu/data" topic and specify the message type (Imu)
    rospy.Subscriber("imu/data", Imu, imu_data_callback)

    # Keep the node running until it is manually stopped
    rospy.spin()

try:
    imu_listener()
except rospy.ROSInterruptException:
    pass

