//
// Created by demphi on 2021/10/1.
//

#ifndef _PACEWORKER_H
#define _PACEWORKER_H

#endif 

class PaceWorker:public StateWorker{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_angle;
    std_msgs::Float64MultiArray msg_angle;

    virtual void run();
    virtual bool is_finished();

    PaceWorker(ros::NodeHandle &nh);
    ~PaceWorker();
};

PaceWorker::PaceWorker(ros::NodeHandle &nh){
    this->nh = nh;
}

PaceWorker::~PaceWorker() {

}

void PaceWorker::run() {

}

bool PaceWorker::is_finished(){

}