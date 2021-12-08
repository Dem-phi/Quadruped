//
// Created by demphi on 2021/10/1.
//

#ifndef _STANDWORKER_
#define _STANDWORKER_

#include "StateWorker.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64MultiArray.h"

/**
 * @brief send angle msg and stand
 * @param none
 */
class StandWorker: public StateWorker{
private:
    /*! max = pi/6*/
    int flag_ = 0;
    double command_hip_ = 0;
    double command_knee_ = 0;

public:
    ros::NodeHandle nh_;
    ros::Publisher angle_real_pub_;

    std_msgs::Float64MultiArray angle_real_data_;
    std_msgs::Float64MultiArray angle_real_data_last_;

    void

    virtual void run();
    virtual bool is_finished();

    StandWorker(ros::NodeHandle &nh);
    ~StandWorker();

};

StandWorker::StandWorker(ros::NodeHandle &nh) {
    this->nh_ = nh;
    this->angle_real_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle", 10
            );

    /*! Init some param*/
    this->angle_real_data_.data.resize(12);
    this->angle_real_data_last_.data.resize(12);
}

StandWorker::~StandWorker() {

}

void StandWorker::run() {
    ROS_INFO("Standing");
    if(this->command_hip_>1 && this->flag_ == 0){this->flag_ = 1;}

    this->angle_real_data_.data[0] = 0.0;
    this->angle_real_data_.data[1] = this->command_hip_*M_PI/6*quad::Rad2Deg;
    this->angle_real_data_.data[2] = 0.0;
    if(this->flag_ ==  0){
        this->command_hip_ = this->command_hip_ + 0.01;
    }
    this->angle_real_pub_.publish(this->angle_real_data_);

    /*! for safety */
    this->angle_real_data_last_ = this->angle_real_data_;
}

bool StandWorker::is_finished() {
    return false;
}

#endif
