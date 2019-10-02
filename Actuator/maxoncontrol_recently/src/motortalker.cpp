//sudo ./jetson_clocks.sh 
//sudo slcand -o -c -f -s4 /dev/ttyUSB3 slcan0
//sudo ifconfig slcan0 up
//candump slcan0
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "msgpkg/Val.h"
#include "msgpkg/realVal.h"
#include "msgpkg/msgmag.h"
#include <termios.h>
#include <stdio.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>

#define M_PI 3.14159265358979323846
float pulse_ratio = 524287.0/(2*M_PI);//1000.0*4.0*14976.0/175.0/(2*M_PI);

float hz = 1.0/500;
int rev = 0.0;
float pre = 0.0; 
int radtopulse(float rad){
    
    int pulse = int(rad*pulse_ratio);
    return pulse;
}

int signfunc(float data){
if (data >= 0) return 1;
if (data < 0) return -1;
}

float des_pos_trajectory(int cnt){
float dxl_goal_position;
/*for (int i = 1; i <= 41; i++) {


	if (cnt < 5*i/hz && cnt >= 5*(i-1)/hz){
 //ROS_INFO("%d",cnt);
	dxl_goal_position = 0.0000000+(0.3)*((i-1)/10.0);
if (cnt >= 11*5/hz)  dxl_goal_position = (0.3)+(-0.3)*((i-11)/10.0);
if (cnt >=31*5/hz) dxl_goal_position = (-0.3)+(0.3)*((i-31)/10.0);

}

}
if (cnt >=41*5/hz) dxl_goal_position = 0.00;*/
/*if (cnt <5/hz) dxl_goal_position = 0.00;
else {
	dxl_goal_position = (0.3)*sin(2*M_PI*0.5*(cnt-5/hz)*hz);
	if (signfunc(dxl_goal_position) != signfunc(pre)) rev = rev+1;
}
if (rev >= 9){
	dxl_goal_position = 0.0000000;
}*/
    if (cnt < 10/hz) {
	dxl_goal_position = 0;
}
    else if (cnt < 20/hz) {
	dxl_goal_position = 1000;
}
    else if (cnt < 30/hz) {
	dxl_goal_position = 2000;
}
    else if (cnt < 40/hz) {
	dxl_goal_position = 1000;
}
    else if (cnt < 50/hz) {
	dxl_goal_position = 0;
}
    else if (cnt < 60/hz) {
	dxl_goal_position = -1000;
}
    else if (cnt < 70/hz) {
	dxl_goal_position = -2000;
}
    else if (cnt < 80/hz) {
	dxl_goal_position = -1000;
}
    else {
	dxl_goal_position = 0;
}

            pre = dxl_goal_position;
    return dxl_goal_position;

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "motortalker");

    ros::NodeHandle n;

    ros::Publisher command_pub = n.advertise<msgpkg::Val>("command", 1);
    msgpkg::Val msg;
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
	msg.position = (desired_pos);

	//ROS_INFO("%d\n", msg.position);

        command_pub.publish(msg);

        ros::spinOnce();

        pre = begin;
        ++count;
	}

    }
	
    return 0;
}
