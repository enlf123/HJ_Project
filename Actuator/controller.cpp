#include <stdio.h>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cmath>

float X[3];
float A_k[3][3] = {{-30.41, 76.83,328.32},{-73.16, -968.23,-959.20},{1.31, 7.76, -122.06}};
float B_k[3][2] ={{0,0.0006},{0,-0.0073},{0,-48.32}};

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

int main()
{
	RK_solver x;
	int total_time = 10.0;
	float dt = 0.001;
	float t[int(total_time/dt)+1];
	float Y[3] = {2,1,1};
//	float pre_Y[3];
//	for (int i=0;i<=sizeof(Y)/sizeof(*Y);i++){
//		pre_Y[i] = Y[i];
//	}
//	
//	for (int i=0;i<=sizeof(t)/sizeof(*t); i++){
//		t[i] = dt*i;
//	}
	
	for (int i=0;i<=sizeof(t)/sizeof(*t); i++){
		x = RK4(Y[0],Y[1],Y[2],0,0,0.0,dt);
		Y[0] = x.y1;
		Y[1] = x.y2;
		Y[2] = x.y3;
		printf("%f,   %f,    %f \n", x.y1, x.y2, x.y3);
	}

	return 0;
}


