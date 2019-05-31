#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "maxoncontrol/Val.h"
#include "maxoncontrol/realVal.h"
#include <termios.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "motortalker");

    ros::NodeHandle n;

    ros::Publisher command_pub = n.advertise<maxoncontrol::Val>("command", 1);
    maxoncontrol::Val msg;
    int arraySize = 0;
	std::string s;
    std::cout << "Motor Talker running \n";
	ros::Rate loop_rate(500);
    int count = 0;

    while (ros::ok())
    {

        if (count < 500*10){
		msg.position = 0;
	}
	else {
		msg.position = count*100-500*10*100;
	}	 

	//ROS_INFO("%d\n", msg.position);

        command_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;

    }
	
    return 0;
}
