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

#define M_PI 3.14159265358979323846
float pulse_ratio = 1000.0*4.0*14976.0/175.0/(2*M_PI);

float hz = 1.0/400;

int radtopulse(float rad){
    
    int pulse = int(rad*pulse_ratio);
    return pulse;
}

float des_pos_trajectory(int cnt){
float dxl_goal_position;
    if (cnt < 5/hz){
        dxl_goal_position = 0.0;
}
else{
	dxl_goal_position = 2*M_PI*sin(0.2*(cnt-5/hz)*hz);
}
/*    else if (cnt < 10/hz) {
	dxl_goal_position = M_PI/2;
}
    else if (cnt < 15/hz) {
	dxl_goal_position = M_PI;
}
    else if (cnt < 20/hz) {
	dxl_goal_position = M_PI*3/2;
}
    else if (cnt < 25/hz) {
	dxl_goal_position = M_PI*2;
}
    else if (cnt < 30/hz) {
	dxl_goal_position = M_PI*3/2;
}
    else if (cnt < 35/hz) {
	dxl_goal_position = M_PI;
}
    else if (cnt < 40/hz) {
	dxl_goal_position = M_PI*1/2;
}
    else if (cnt < 45/hz) {
	dxl_goal_position = 0;
}
    else if (cnt < 50/hz) {
	dxl_goal_position = -M_PI/2;
}
    else if (cnt < 55/hz) {
	dxl_goal_position = -M_PI;
}
    else if (cnt < 60/hz) {
	dxl_goal_position = -M_PI*3/2;
}
    else if (cnt < 65/hz) {
	dxl_goal_position = -M_PI*2;
}
    else if (cnt < 70/hz) {
	dxl_goal_position = -M_PI*3/2;
}
    else if (cnt < 75/hz) {
	dxl_goal_position = -M_PI;
}
    else if (cnt < 80/hz) {
	dxl_goal_position = -M_PI*1/2;
}
    else {
	dxl_goal_position = 0;
}*/

            
    return float(dxl_goal_position);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motortalker");

    ros::NodeHandle n;

    ros::Publisher command_pub = n.advertise<maxoncontrol::Val>("command", 1);
    maxoncontrol::Val msg;
    int arraySize = 0;
    float desired_pos = 0;
	std::string s;
    std::cout << "Motor Talker running \n";
    ros::Rate loop_rate(1.0/hz);
    int count = 0;
    ros::Time pre = ros::Time::now();
    while (ros::ok())
    {

	ros::Time begin = ros::Time::now(); 
        if ((begin-pre).toSec()>hz){
        desired_pos = des_pos_trajectory(count);
	msg.position = radtopulse(desired_pos);

	//ROS_INFO("%d\n", msg.position);

        command_pub.publish(msg);

        ros::spinOnce();

        pre = begin;
        ++count;
	}

    }
	
    return 0;
}
