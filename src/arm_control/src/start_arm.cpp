#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

// Integer codes given to each key
using namespace ros;
using namespace std;

string input;

int main(int argc, char **argv)
{
    init(argc, argv, "publisher_node");

    NodeHandle handle;

    Publisher pub = handle.advertise<std_msgs::String>("Movement", 10); // creating a publisher node

    Rate rate(5); // 5 Hz, ie , 0.2 seconds

    while (ok()) // User not pressed Q and ros is 'ok' (no problem to run)
    {
        std_msgs::String msg;

        cout<<"Enter Angles : ";

        cin>>input;

        msg.data = input;

        pub.publish(msg); // publishing the integer code of key to all subscriber
        rate.sleep();     // refresh rate
    }


    return 0;
}
