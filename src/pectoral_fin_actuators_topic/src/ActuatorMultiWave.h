/*
Filename: ActuatorMultiWave.h
Description: object that contains the important information for operating the stingray Servos. Operates using an 
array for each actuator
Author: Alex Cunningham alc5@hw.ac.uk
Start date: 01/12/2021
Last update: 01/12/2021
*/
#pragma once
#include <cmath>
#include "PCA9685.h"
#include "Servo.h"
#include <vector>
#include <array>
///Class that holds servo objects and important servo details
///this class does not contain the servo offsets
///@tparam N number of servos
///@param int delayTime
//TODO: check output for waveVector
//TODO: add smooth transition when changing between waves
//TODO: add explicit pin choice to constructor for initializing memory location 
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
    double numberServos_;
    public:
    ~ActuatorMultiWave(){
        setToSafePos();
	for (auto servoIt = this->begin();servoIt!=this->end();++servoIt){
             // delete all servos
             delete *servoIt;
             }
	
    }
    ActuatorMultiWave(bool isRightActuator,double numberServos,double delayTime,double resolution,double winderRadius,double rayThickness,double alphaLink,double frequency){
	 //if isRightActuator then right hand size address buses are used, this is for the bottom servo hat stack
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
	 }
	 else{
	 	driver.init(1,0x41);
		driver.setPWMFreq(45);
	 }	 
         int pin = 2;
	 int channelIterator=0;
         for (auto servoIt = this->begin();servoIt!=this->end();++servoIt){
             // Attach the servos
             servoIt*=new Servo(channelIterator,&driver,65,115);
             channelIterator++;
             }
         }

    //update servoWaveVector alpha(t)=Asin(2pift)
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
        for(auto servoIterator=this->numberServos_;servoIterator!=this->numberServos_;servoIterator++){
            while (tIterator < 1/(this->frequency_)){
                double alphaLinkIterator = (pi/180.00)*(this->alphaLink_)*sin((this->frequency_+phaseDif*actuatorNum)*2*pi*tIterator);
                double deltaL=4*(this->rayThickness_)*sin(alphaLinkIterator/2);
                tmpWaveContainer.push_back(90.0-((deltaL/(this->winderRadius_))*180.00/pi)); //based on a 90 degree neutral position
                waveVector.push_back(tmpWaveContainer);
                tIterator += this->delayTime_;
                tmpWaveContainer.clear();
            }
        actuatorNum++;
        }    
        
    }
    
    //update servoWaveVector alpha(t)=Asin(2pift) if no args passed set to pos given by member variables
    void setWaveArray() {
        float pi = 3.14159265358979;
        auto tmpWaveContainer=std::vector<T>();
        tmpWaveContainer.push_back(90.0);
        //waveVector.begin().push_back(90.0);
        double tIterator = this->delayTime_;
        double phaseDif=this->frequency_*2*pi/this->numberServos_/2;
        int actuatorNum = 0;
        for(auto servoIterator=this->numberServos_;servoIterator!=this->numberServos_;servoIterator++){
            while (tIterator < 1/(this->frequency_)){
                double alphaLinkIterator = (pi/180.00)*(this->alphaLink_)*sin((this->frequency_+phaseDif*actuatorNum)*2*pi*tIterator);
                double deltaL=4*(this->rayThickness_)*sin(alphaLinkIterator/2);
                tmpWaveContainer.push_back(90.0-((deltaL/(this->winderRadius_))*180.00/pi)); //based on a 90 degree neutral position
                waveVector.push_back(tmpWaveContainer);
                tIterator += this->delayTime_;
                tmpWaveContainer.clear();
            }
        actuatorNum++;
        }    
    }

    //setServosToPosition 
    void setServosToPosition(){
        int servoIndex=0;
        for (auto servoIt = this->begin();servoIt!=this->end();servoIt++){
            servoIt->write(this->waveVector[0][int(this->waveVector[0].size()*servoIndex/this->numberServos_/2)]);
            servoIt++;
            servoIndex+=1;
            }
        return;
    }
    //setServosToSafePosition - used within destructor
    
    void setToSafePos(){
        for (auto servoIt = this->begin();servoIt!=this->end();servoIt++){
            servoIt->write(80);
            }
        return;
    }

    void waveServos(){
        for (int waveIndex=0;waveIndex!=waveVector.size();waveIndex++){
            for (auto actuatorIterator=this->begin();actuatorIterator!=this->end();actuatorIterator++){
                int actuatorPos=(actuatorIterator-this->begin())/2;
                actuatorIterator->write(waveVector[actuatorPos][waveIndex]);
                actuatorIterator++;
                actuatorIterator->write(90.0-(waveVector[actuatorPos][waveIndex]-90.0));
            }
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
