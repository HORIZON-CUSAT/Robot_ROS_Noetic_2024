#!/usr/bin/env python
import rospy
from std_msgs.msg import String

# Define the available steering modes
steering_modes = [
    "Differential Steering",
    "Crab Turn",
    "Zero Turn",
    "4 Wheel Steer",
    "2 Wheel Steer",
    "Exit"
]

def display_menu():
    print("Select a steering mode:")
    for idx, mode in enumerate(steering_modes):
        print("{}. {}".format(idx + 1, mode))
    return input("Enter the number of your choice: ")

def main():
    rospy.init_node('steering_mode_publisher', anonymous=True)
    steering_mode_pub = rospy.Publisher('steering_mode', String, queue_size=10)

    while not rospy.is_shutdown():
        choice = display_menu()

        try:
            choice = int(choice)
        except ValueError:
            print("Invalid input. Please enter a number.")
            continue

        if 1 <= choice <= len(steering_modes) - 1:
            selected_mode = steering_modes[choice - 1]
            rospy.loginfo("Selected steering mode: {}".format(selected_mode))
            steering_mode_pub.publish(selected_mode)
        elif choice == len(steering_modes):
            rospy.loginfo("Exiting the steering mode selection.")
            break
        else:
            print("Invalid choice. Please enter a valid number.")

try:
    main()
except rospy.ROSInterruptException:
    pass

