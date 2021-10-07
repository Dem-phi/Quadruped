//
// Created by demphi on 2021/10/1.
//

#ifndef _GALLOPWORKER_H
#define _GALLOPWORKER_H

#include "CAR.h"
#include "GaitTable.h"

class GallopWorker:public StateWorker{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_angle;
    std_msgs::Float64MultiArray msg_angle;

    CAR* model_CAR = new CAR(0.3, 0.02, 0.75);

    virtual void run();
    virtual bool is_finished();

    GallopWorker(ros::NodeHandle &nh);
    ~GallopWorker();

};

GallopWorker::GallopWorker(ros::NodeHandle &nh) {
    this->nh = nh;
}

GallopWorker::~GallopWorker() {

}

void GallopWorker::run() {

}

bool GallopWorker::is_finished() {
    return false;
}

#endif
