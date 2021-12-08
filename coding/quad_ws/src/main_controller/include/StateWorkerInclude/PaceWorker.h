//
// Created by demphi on 2021/10/1.
//

#ifndef _PACEWORKER_H
#define _PACEWORKER_H

#include "CAR.h"
#include "Hopf.h"
#include "GaitTable.h"

class PaceWorker:public StateWorker{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_angle;
    std_msgs::Float64MultiArray msg_angle;

    CAR* model_CAR = new CAR(1.2, 0.4, 0.02, quad::PACE_BETA);
    Hopf* model_Hopf = new Hopf({10.0, 5.3}, quad::PACE_BETA, quad::PACE_PHI);
    Eigen::Vector2f amplitude;

    virtual void run();
    virtual bool is_finished();

    PaceWorker(ros::NodeHandle &nh);
    ~PaceWorker();
};

PaceWorker::PaceWorker(ros::NodeHandle &nh){
    this->nh = nh;
    this->pub_angle = this->nh.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle", 1);
    this->amplitude = model_CAR->Calculate_amplitude();
    //init
    this->msg_angle.data.resize(12);
}

PaceWorker::~PaceWorker() {
    delete this->model_CAR;
}

void PaceWorker::run() {
    ROS_INFO("Pacing");
    std::cout << amplitude.x() << "   " << amplitude.y() << std::endl;
}

bool PaceWorker::is_finished(){
    return false;
}

#endif