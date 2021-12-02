/*
 * Filename: actuator.cpp
 * Description: this code controls both pectoral actuators 
 * Author: Alex Cunningham alexanderlewiscunningham@gmail.com
 * Start date: 01/12/2021
 * Last update: 02/12/2021
*/
#include <ros/ros.h>
#include "ActuatorMultiWave.h"
int main(int argc,char** argv){
	ros::NodeHandle* nh=NULL;
	ros::init(argc,argv,"pectoral_fin_actuators_node");
	nh=new ros::NodeHandle();
	if(!nh){
		ROS_FATAL("Failed to initialize NodeHandle");
		ros::shutdown();
		return -1;
	}

	ros::Rate rate(100);
	//setup Actuator
	//amplitude is related by alphai=cos^-1(A/(2L1+3L2)) where alpha < 15
	ActuatorMultiWave <2,double>actuator(true,2,0.02,0.18,0.01,0.007,15.0,2.0);	
	actuator.setWaveArray();
	//actuator.setServosToPosition();
	while(ros::ok()){
		//actuator.waveServos();
		int number=1;	
	}
	delete nh;
	return 0;
}
