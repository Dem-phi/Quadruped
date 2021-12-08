//
// Created by demphi on 2021/10/4.
//

#ifndef _INITCHECK_
#define _INITCHECK_

#include "StandWorker.h"
#include "std_msgs/Float64MultiArray.h"

class InitCheck:public StateWorker{
private:
    ros::Publisher angle_real_pub_;
public:
    ros::NodeHandle nh_;

    std_msgs::Float64MultiArray angle_real_data_;

    virtual void run();
    virtual bool is_finished();
    InitCheck(ros::NodeHandle &nh);
    ~InitCheck();
};
InitCheck::InitCheck(ros::NodeHandle &nh) {
    this->nh_ = nh;
    this->angle_real_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle", 1);

    /*! Init some param */
    this->angle_real_data_.data.resize(12);
}

InitCheck::~InitCheck() {

}

void InitCheck::run() {
    ROS_INFO("Init check!");

    /*!
     * need to add function for judging
     * */


    this->angle_real_data_.data[0] = 0;
    this->angle_real_data_.data[1] = 0.0;
    this->angle_real_data_.data[2] = 0.0;
    this->angle_real_pub_.publish(this->angle_real_data_);
}

bool InitCheck::is_finished() {
    return false;
}

#endif 
