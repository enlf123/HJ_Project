#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "maxoncontrol/Val.h"
#include "maxoncontrol/realVal.h"
#include <sstream>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include <iomanip>


#include <net/if.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

float torqueValue;

#define CANID_DELIM '#'
#define DATA_SEPERATOR '.'
#define M_PI 3.14159265358979323846
typedef void* HANDLE;
typedef int BOOL;

using namespace std;

void* g_pKeyHandle = 0;
int vel = 0;
int cur = 0;
int pos = 0;

unsigned short g_usNodeId[] = {1};
int Id_length = sizeof(g_usNodeId)/sizeof(*g_usNodeId);

// CAN Parameters
int s;
string Control_Enable_vel = "0F00";
string Control_Enable_pos = "3F00"; //relative position
string Control_Enable_cur = "";
unsigned short COB_ID_Rx[][4] = {{0x201, 0x301, 0x401, 0x501}};
                              
string COB_ID_Tx[][4] = {{"181", "281", "381", "481"}};
                       


float X[3];
float Y[3] = {10,1,1};
float A_k[3][3] = {{-30.41, 76.83,328.32},{-73.16, -968.23,-959.20},{1.31, 7.76, -122.06}};
float B_k[3][2] ={{0,0.0006},{0,-0.0073},{0,-48.32}};


float pulse_ratio = 1000.0*4.0*14976.0/175.0/(2*M_PI);

int radtopulse(float rad){
    
    int pulse = int(rad*pulse_ratio);
    return pulse;
}

float pulsetorad(int pulse){
    
    float rad = float(float(pulse)/pulse_ratio);
    return rad;
}

typedef struct _RK_solver
{
float y1;
float y2;
float y3;
}RK_solver;


float func1(float x1,float x2, float x3,float z1,float z2)
{
	float Value = A_k[0][0]*x1+A_k[0][0]*x2+A_k[0][2]*x3+B_k[0][0]*z1+B_k[0][1]*z2;
	
	return Value;
}

float func2(float x1,float x2, float x3,float z1,float z2)
{
	float Value = A_k[1][0]*x1+A_k[1][1]*x2+A_k[1][2]*x3+B_k[1][0]*z1+B_k[1][1]*z2;
	
	return Value;
}

float func3(float x1,float x2, float x3,float z1,float z2)
{
	float Value = A_k[2][0]*x1+A_k[2][1]*x2+A_k[2][2]*x3+B_k[2][0]*z1+B_k[2][1]*z2;
	
	return Value;
}

RK_solver RK4(float x1,float x2, float x3,float z1,float z2, float t1, float t2)
{
	RK_solver value;
	float k[3][4]; 
	int n = 40.0;
	float dt = (t2-t1)/n;
	float Y[n][3];
	Y[0][0]= x1; 
	Y[0][1]= x2; 
	Y[0][2]= x3; 
	for (int i=0; i<n-1; i++){
		k[0][0] = func1(Y[i][0],Y[i][1],Y[i][2],z1,z2);
		k[0][1] = func1(Y[i][0]+0.5*dt*k[0][0],Y[i][1]+0.5*dt*k[0][0],Y[i][2]+0.5*dt*k[0][0],z1,z2);
		k[0][2] = func1(Y[i][0]+0.5*dt*k[0][1],Y[i][1]+0.5*dt*k[0][1],Y[i][2]+0.5*dt*k[0][1],z1,z2);
		k[0][3] = func1(Y[i][0]+dt*k[0][2],Y[i][1]+dt*k[0][2],Y[i][2]+dt*k[0][2],z1,z2);
		k[1][0] = func2(Y[i][0],Y[i][1],Y[i][2],z1,z2);
		k[1][1] = func2(Y[i][0]+0.5*dt*k[1][0],Y[i][1]+0.5*dt*k[1][0],Y[i][2]+0.5*dt*k[1][0],z1,z2);
		k[1][2] = func2(Y[i][0]+0.5*dt*k[1][1],Y[i][1]+0.5*dt*k[1][1],Y[i][2]+0.5*dt*k[1][1],z1,z2);
		k[1][3] = func2(Y[i][0]+dt*k[1][2],Y[i][1]+dt*k[1][2],Y[i][2]+dt*k[1][2],z1,z2);
		k[2][0] = func3(Y[i][0],Y[i][1],Y[i][2],z1,z2);
		k[2][1] = func3(Y[i][0]+0.5*dt*k[2][0],Y[i][1]+0.5*dt*k[2][0],Y[i][2]+0.5*dt*k[2][0],z1,z2);
		k[2][2] = func3(Y[i][0]+0.5*dt*k[2][1],Y[i][1]+0.5*dt*k[2][1],Y[i][2]+0.5*dt*k[2][1],z1,z2);
		k[2][3] = func3(Y[i][0]+dt*k[2][2],Y[i][1]+dt*k[2][2],Y[i][2]+dt*k[2][2],z1,z2);
		Y[i+1][0] = Y[i][0] + (1.0/6.0)*(k[0][0]+2*k[0][1]+2*k[0][2]+k[0][3])*dt;
		Y[i+1][1] = Y[i][1] + (1.0/6.0)*(k[1][0]+2*k[1][1]+2*k[1][2]+k[1][3])*dt;
		Y[i+1][2] = Y[i][2] + (1.0/6.0)*(k[2][0]+2*k[2][1]+2*k[2][2]+k[2][3])*dt;
	}
	value.y1 = Y[n-1][0];
	value.y2 = Y[n-1][1];
	value.y3 = Y[n-1][2];
	return value;
}








unsigned char asc2nibble(char c) {

    if ((c >= '0') && (c <= '9'))
        return c - '0';

    if ((c >= 'A') && (c <= 'F'))
        return c - 'A' + 10;

    if ((c >= 'a') && (c <= 'f'))
        return c - 'a' + 10;

    return 16; /* error */
}

int hexstring2data(char *arg, unsigned char *data, int maxdlen) {

    int len = strlen(arg);
    int i;
    unsigned char tmp;

    if (!len || len%2 || len > maxdlen*2)
        return 1;

    memset(data, 0, maxdlen);

    for (i=0; i < len/2; i++) {

        tmp = asc2nibble(*(arg+(2*i)));
        if (tmp > 0x0F)
            return 1;

        data[i] = (tmp << 4);


        tmp = asc2nibble(*(arg+(2*i)+1));
        if (tmp > 0x0F)
            return 1;

        data[i] |= tmp;
    }

    return 0;
}

int parse_canframe(char *cs, struct canfd_frame *cf) {
    /* documentation see lib.h */

    int i, idx, dlen, len;
    int maxdlen = CAN_MAX_DLEN;
    int ret = CAN_MTU;
    unsigned char tmp;

    len = strlen(cs);
    //printf("'%s' len %d\n", cs, len);

    memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

    if (len < 4)
        return 0;

    if (cs[3] == CANID_DELIM) { /* 3 digits */

        idx = 4;
        for (i=0; i<3; i++){
            if ((tmp = asc2nibble(cs[i])) > 0x0F)
                return 0;
            cf->can_id |= (tmp << (2-i)*4);
        }

    } else if (cs[8] == CANID_DELIM) { /* 8 digits */

        idx = 9;
        for (i=0; i<8; i++){
            if ((tmp = asc2nibble(cs[i])) > 0x0F)
                return 0;
            cf->can_id |= (tmp << (7-i)*4);
        }
        if (!(cf->can_id & CAN_ERR_FLAG)) /* 8 digits but no errorframe?  */
            cf->can_id |= CAN_EFF_FLAG;   /* then it is an extended frame */

    } else
        return 0;

    if((cs[idx] == 'R') || (cs[idx] == 'r')){ /* RTR frame */
        cf->can_id |= CAN_RTR_FLAG;

        /* check for optional DLC value for CAN 2.0B frames */
        if(cs[++idx] && (tmp = asc2nibble(cs[idx])) <= CAN_MAX_DLC)
            cf->len = tmp;

        return ret;
    }

    if (cs[idx] == CANID_DELIM) { /* CAN FD frame escape char '##' */

        maxdlen = CANFD_MAX_DLEN;
        ret = CANFD_MTU;

        /* CAN FD frame <canid>##<flags><data>* */
        if ((tmp = asc2nibble(cs[idx+1])) > 0x0F)
            return 0;

        cf->flags = tmp;
        idx += 2;
    }

    for (i=0, dlen=0; i < maxdlen; i++){

        if(cs[idx] == DATA_SEPERATOR) /* skip (optional) separator */
            idx++;

        if(idx >= len) /* end of string => end of data */
            break;

        if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
            return 0;
        cf->data[i] = (tmp << 4);
        if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
            return 0;
        cf->data[i] |= tmp;
        dlen++;
    }
    cf->len = dlen;

    return ret;
}

void LogInfo(string message)
{
    cout << message << endl;
}

template <typename T>
std::string dec_to_hex(T dec, int NbofByte){
	
	std::stringstream stream_HL;
    string s, s_LH;
	
    stream_HL << std::setfill ('0') << std::setw(sizeof(T)*2) <<std::hex << dec;
    s = stream_HL.str();
    for (int i=0; i<NbofByte; i++){
        s_LH.append(s.substr(2*(NbofByte-1-i),2));
    }
    return s_LH;

}


int hexarray_to_int4(unsigned char *buffer4){
//    int length = sizeof(buffer)/ sizeof(*buffer);
    int length = 4;
    int hextoint = 0;
    for (int i=0; i<length; i++)
    {
        hextoint += (buffer4[i] << 8*i);
    }

    return hextoint;
}

int hexarray_to_int4_2(unsigned char *buffer6){
//    int length = sizeof(buffer)/ sizeof(*buffer);
    int length = 4;
    int hextoint = 0;
    for (int i=0; i<length; i++) //first 4 bytes
    {
		    hextoint += (buffer6[i] << 8*i);
	}

    return hextoint;
}
int hexarray_to_int4_3(unsigned char *buffer6){
//    int length = sizeof(buffer)/ sizeof(*buffer);
    int length = 2;
    int hextoint = 0;
    for (int i=0; i<length; i++) 
    {
		    hextoint += (buffer6[i+4] << 8*i);
	}
    return hextoint;
}

short hexarray_to_int2(unsigned char *buffer2){
//    int length = sizeof(buffer)/ sizeof(*buffer);
    int length = 2;
    short hextoint = 0;
	unsigned char x = 0xff;
    for (int i=0; i<length; i++)
    { 
		hextoint += (buffer2[i] << 8*i);
    }

    return hextoint;
}


std::string stringappend(string a, string b)
{
    string s;
    s.append(a);
    s.append(b);
    return s;
}

int tester = 1;

void commandCallback(const maxoncontrol::Val::ConstPtr& msg)
{

    string data;
    struct can_frame frame;
    string val_desired_hex[Id_length];

    int setVal[] = {msg->position};

    stringstream ss;
    RK_solver x;
    float dt = 0.002;

    x = RK4(Y[0],Y[1],Y[2],0,0,0.0,dt);

   // ROS_INFO("%f , %f, %f", x.y1, x.y2, x.y3 );
    Y[0] = x.y1;
    Y[1] = x.y2;
    Y[2] = x.y3;

    for (int i = 0; i < Id_length; i++) {
        // Convert dec to hex
        val_desired_hex[i] = dec_to_hex(setVal[i], 4);
        // Send position control command to EPOS
        frame.can_id = COB_ID_Rx[i][3];
        frame.can_dlc = 6;
	//Command to locate desired position
        data = stringappend(Control_Enable_pos, val_desired_hex[i]);
	hexstring2data((char *) data.c_str(), frame.data, 8);
        write(s, &frame, sizeof(struct can_frame));
		ros::Duration(0.00001).sleep();
    }

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_pdo");
    ros::NodeHandle n;
	int numOfData = 3;
    string rtr;
    unsigned char buffer[4];
	unsigned char buffer2[2];
	unsigned char buffer4[4];
	unsigned char buffer6[6];

    // SocketCAN Initialize
    struct sockaddr_can addr;
    struct can_frame frame;
    struct canfd_frame frame_fd;
    int required_mtu;
    struct ifreq ifr;

    struct iovec iov;
    struct msghdr canmsg;
    char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
    struct canfd_frame frame_get;
	
    const char *ifname = "slcan0";

    if((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    printf("%s at index %d\n", ifname, ifr.ifr_ifindex);
  

//    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    if(bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    // Initialize NMT Services
    LogInfo("Initialize NMT Services");
    // Reset Communication
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x81;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Reset Communication");
    sleep(1);

	recvmsg(s, &canmsg, 0);

    // Start Remote Node
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x01;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Start Remote Node");
    sleep(1);

    // Control mode Initialize
    for (int i=0; i<Id_length; i++)
    {
	frame.can_id  = COB_ID_Rx[i][1];
        frame.can_dlc = 3;
        frame.data[0] = 0x00;
        frame.data[1] = 0x00;

        frame.data[2] = 0x01;


        write(s, &frame, sizeof(struct can_frame));
        sleep(0.8);
        

        // Shutdown Controlword
        frame.can_id  = COB_ID_Rx[i][0];
        frame.can_dlc = 2;
        frame.data[0] = 0x06;
        frame.data[1] = 0x00;
        write(s, &frame, sizeof(struct can_frame));
        sleep(1);

        // Enable Controlword
        frame.can_id  = COB_ID_Rx[i][0];
        frame.can_dlc = 2;
        frame.data[0] = 0x0F;
        frame.data[1] = 0x00;
        write(s, &frame, sizeof(struct can_frame));
        sleep(1);
    }

    ros::Subscriber sub = n.subscribe("command", 1, commandCallback);
    ros::Publisher position_pub = n.advertise<maxoncontrol::realVal>("position", 1);

    ros::Rate loop_rate(400);

    iov.iov_base = &frame_get;
    canmsg.msg_name = &addr;
    canmsg.msg_iov = &iov;
    canmsg.msg_iovlen = 1;
    canmsg.msg_control = &ctrlmsg;

    iov.iov_len = sizeof(frame_get);
    canmsg.msg_namelen = sizeof(addr);
    canmsg.msg_controllen = sizeof(ctrlmsg);
    canmsg.msg_flags = 0;
	sleep(1);

	int nnbytes = 0;
	float tmp = 0;
	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 15;
	setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

	maxoncontrol::realVal msg;

	ros::Duration(0.0001).sleep();

    ros::Time pre = ros::Time::now();
    while (ros::ok())
    {
	stringstream ss;
	ros::Time begin = ros::Time::now();
        if ((begin-pre).toSec()>0.0025){   	
        	for (int i=0; i<Id_length;i++) {
            		rtr = stringappend(COB_ID_Tx[i][3], "#R");//read position
            		required_mtu = parse_canframe((char*)(rtr.c_str()), &frame_fd);
	                write(s, &frame_fd, required_mtu);
            		nnbytes = recvmsg(s, &canmsg, 0);
            		if ((size_t) nnbytes != CAN_MTU && (size_t) nnbytes != CANFD_MTU) {}
            		else {
                		msg.realPos = pulsetorad(hexarray_to_int4(frame_get.data));
                //ROS_INFO("%f", tmp);
            			}
        	}
		
               // ROS_INFO("%f", (begin-pre).toSec());
		position_pub.publish(msg);
		ros::Duration(0.000005).sleep();
		pre = begin;
	 }
        ros::spinOnce();
	
        //loop_rate.sleep();

    }

    // Reset Remote Node
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x81;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Reset Remote Node");
	sleep(0.2);
    // Stop Remote Node
    frame.can_id  = 0x000;
    frame.can_dlc = 2;
    frame.data[0] = 0x02;
    frame.data[1] = 0x00;
    write(s, &frame, sizeof(struct can_frame));
    LogInfo("Stop Remote Node");


    return 0;
}
