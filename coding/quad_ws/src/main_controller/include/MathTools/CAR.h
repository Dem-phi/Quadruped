//
// Created by demphi on 2021/10/1.
//

#ifndef _CAR_
#define _CAR_

#include "reference/common_include.h"

using namespace Eigen;
using namespace std;

class CAR{
public:
    //theta0 is the start angle
    const float theta0 = 30;
    float beta = 0;
    float step_size = 0;
    float leg_length = 0.4, thigh_length = 0.4;
    float period = 0.8;
    float foot2ground = 0;
    float velocity = 0;
    //rad to deg -> deg = rad * Rad2Deg
    //deg to rad -> rad = deg / Rad2Deg
    double Rad2Deg = 45.0/ atan(1.0);

    Eigen::Vector2f amplitude = {0.0, 0.0};

    CAR(float velocity, float period, float foot2ground, float beta);
    ~CAR();
    Eigen::Vector2f Calculate_amplitude();
};

CAR::CAR(float velocity, float period, float foot2ground, float beta) {
    this->period = period;
    this->velocity = velocity;
    this->step_size = this->velocity * this->period;
    this->beta = beta;
    this->foot2ground = foot2ground;
}

CAR::~CAR() {

}

Eigen::Vector2f CAR::Calculate_amplitude(){
    this->amplitude.x() = asin((this->step_size*this->beta)/(4*this->thigh_length*cos(this->theta0/this->Rad2Deg ))) * this->Rad2Deg;
    this->amplitude.y() = acos((this->leg_length* cos(this->theta0/this->Rad2Deg)-this->foot2ground)/this->leg_length)*this->Rad2Deg - this->theta0;
    return this->amplitude;
}



#endif 
