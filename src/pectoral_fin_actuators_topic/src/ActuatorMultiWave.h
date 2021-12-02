/*
Filename: ActuatorMultiWave.h
Description: object that contains the important information for operating the stingray Servos. Operates using an 
array for each actuator
Author: Alex Cunningham alc5@hw.ac.uk
Start date: 01/12/2021
Last update: 02/12/2021
*/
#pragma once
#include <cmath>
#include "PCA9685.h"
#include "Servo.h"
#include <vector>
#include <array>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
///Class that holds servo objects and important servo details
///this class does not contain the servo offsets
///@tparam N number of servos
///@param int delayTime
//TODO: check output for waveVector
//TODO: check code for second stack of servos google "coding stacked PCA9685"
template<unsigned int N,typename T>
class ActuatorMultiWave {
    private:
    constexpr static unsigned int numServos=N;
    Servo servos[N];
    PCA9685 driver = PCA9685();
    double delayTime_;
    std::vector<std::vector<T>> waveVector;
    double resolution_;
    double winderRadius_;
    double rayThickness_;
    double alphaLink_;
    double frequency_; 
    int numberServos_;
    std_msgs::String actuatorSide_;
    public:
    ~ActuatorMultiWave(){
        setToSafePos();
	//for (auto servoIt = this->begin();servoIt!=this->end();++servoIt){
             // delete all servos
        //     delete servoIt;
         //    }
	
    }
    ActuatorMultiWave(bool isRightActuator,int numberServos,double delayTime,double resolution,double winderRadius,double rayThickness,double alphaLink,double frequency){
	 //if isRightActuator then right hand size address buses are used, this is for the bottom servo hat stack
	 if(isRightActuator){
	 	actuatorSide_.data="right";
	 }
	 else{
		actuatorSide_.data="left";
	 } 
	 ROS_INFO_STREAM("Actuator connected on " << this->actuatorSide_);
	 numberServos_ =numberServos; 
         delayTime_=delayTime;
         waveVector=std::vector<std::vector<T>>();
         resolution_=resolution;
         winderRadius_=winderRadius;
         rayThickness_=rayThickness/2;
         //alphaLink_ in radians
         alphaLink_=alphaLink;
         frequency_=frequency;
	 //set driver and memory locations
	 //right actuator is connected to the bottom servo hat 
	 if (isRightActuator){
		driver.init(1,0x40);
		driver.setPWMFreq(45);
		ROS_INFO_STREAM("initialized right actuator driver");
	 }
	 else{
	 	driver.init(1,0x41);
		driver.setPWMFreq(45);
		ROS_INFO_STREAM("initialized left actuator driver");

	 }	 
	 int channelIterator=0;
         for (auto servoIt = this->begin();servoIt!=this->end();++servoIt){
             // Attach the servos
	     ROS_INFO_STREAM("initializing servos");
	     //servoIt=new Servo(channelIterator,&driver,65,115);
	     *servoIt=Servo(channelIterator,&driver,65,115);
	     ROS_INFO_STREAM("initialized...");
             channelIterator++;
	     if (channelIterator > 16){
	     	ROS_ERROR_STREAM("Channels do not exist past 16 eeek");
		ros::shutdown();
	     }
             }
	 ROS_INFO_STREAM("Actuator initialized "<< actuatorSide_);
         }

    //update servoWaveVector alpha(t)=Asin(2pift)
    //TODO; update setwaveArray no.2
    void setWaveArray(int alphaLink,float frequency) {
        //if actuators already producing wave provided then return 
        if (this->alphaLink_ == alphaLink && this->frequency_ ==frequency){
            return;
        }
        this->alphaLink_=alphaLink;
        this->frequency_=frequency;
        //contains same logic as setWaveArray() with noargs 
        float pi = 3.14159265358979;
        auto tmpWaveContainer=std::vector<T>();
        tmpWaveContainer.push_back(90.0);
        //waveVector.begin().push_back(90.0);
        double tIterator = this->delayTime_;
        double phaseDif=this->frequency_*2*pi/this->numberServos_/2;
        int actuatorNum = 0;
        for(auto servoIterator=0;servoIterator!=this->numberServos_/2;servoIterator++){
            while (tIterator < 1/(this->frequency_)){
                double alphaLinkIterator = (pi/180.00)*(this->alphaLink_)*sin((this->frequency_+phaseDif*actuatorNum)*2*pi*tIterator);
                double deltaL=4*(this->rayThickness_)*sin(alphaLinkIterator/2);
                tmpWaveContainer.push_back(90.0-((deltaL/(this->winderRadius_))*180.00/pi)); //based on a 90 degree neutral position
                waveVector.push_back(tmpWaveContainer);
                tIterator += this->delayTime_;
                tmpWaveContainer.clear();
            }
	waveVector.push_back(tmpWaveContainer);
	tmpWaveContainer.clear();
        actuatorNum++;
        }
    }
    
    //update servoWaveVector alpha(t)=Asin(2pift) if no args passed set to pos given by member variables
    void setWaveArray() {
        float pi = 3.14159265358979;
	ROS_INFO_STREAM("SETTING WAVE ARRAY");
        auto tmpWaveContainer=std::vector<T>();
        tmpWaveContainer.push_back(90.0);
        //waveVector.begin().push_back(90.0);
        double tIterator = this->delayTime_;
        double phaseDif=this->frequency_*2*pi/this->numberServos_/2;
        int actuatorNum = 0;
        for(auto servoIterator=0;servoIterator!=this->numberServos_/2;servoIterator++){
		ROS_INFO_STREAM("INSIDE FOR LOOP....");
            while (tIterator < 1/(this->frequency_)){
		//calculate wave for servo 
                double alphaLinkIterator = (pi/180.00)*(this->alphaLink_)*sin((this->frequency_+phaseDif*actuatorNum)*2*pi*tIterator);
                double deltaL=4*(this->rayThickness_)*sin(alphaLinkIterator/2);
                tmpWaveContainer.push_back(90.0-((deltaL/(this->winderRadius_))*180.00/pi)); //based on a 90 degree neutral position
		ROS_INFO("%f",90.0-((deltaL/(this->winderRadius_))*180.00/pi));
                tIterator += this->delayTime_; 	
            }
		waveVector.push_back(tmpWaveContainer);
		tmpWaveContainer.clear();

        actuatorNum++;
        }
	ROS_INFO_STREAM("WAVE ARRAY SET");
    }

    //setServosToPosition 
    void setServosToPosition(){
        int servoIndex=0;
	ROS_INFO_STREAM("setServosToPosition()");
        for (auto servoIt = this->begin();servoIt!=this->end();servoIt++){
            servoIt->setAngle(this->waveVector[0][int(this->waveVector[0].size()*servoIndex/this->numberServos_/2)]);
            servoIt++;
            servoIndex+=1;
            }
	ROS_INFO_STREAM("servos set to position");
        return;
    }
    //setServosToSafePosition - used within destructor
    
    void setToSafePos(){
        for (auto servoIt = this->begin();servoIt!=this->end();servoIt++){
            servoIt->setAngle(80);
            }
        return;
    }

    void waveServos(){
        for (int waveIndex=0;waveIndex!=waveVector.size();waveIndex++){
            for (auto actuatorIterator=this->begin();actuatorIterator!=this->end();actuatorIterator++){
                int actuatorPos=(actuatorIterator-this->begin())/2;
                actuatorIterator->setAngle(waveVector[actuatorPos][waveIndex]);
                actuatorIterator++;
                actuatorIterator->setAngle(90.0-(waveVector[actuatorPos][waveIndex]-90.0));
            }
	//sleep in mircoseconds
	usleep(1000000*(this->delayTime_));
        }
    }
    
    ///access element servo n
    ///@return Servo&
    Servo& operator[](unsigned int n) {return servos[n];}
    ///access element servo n
    ///@return Servo& 
    const Servo& operator[](unsigned int n) const {return servos[n];}
    using iterator = Servo*;

    ///get first servo
    ///@return iterator to start of servos
    iterator begin() {return &servos[0];}
    
    //get last servo
    ///@return iterator to end of servos
    iterator end() {return &servos[N];}
};
