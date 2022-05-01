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
    double init_time_, cur_time_, end_time_;
    double init_hip_ = 90.0, init_knee_ = -175.0;

public:
    ros::NodeHandle nh_;
    ros::Publisher angle_real_pub_, angle_gazebo_pub_;

    std_msgs::Float64MultiArray angle_real_data_, angle_gazebo_data;


    virtual void run(STATE_INTERIOR *cur_state);
    virtual bool is_finished();

    StandWorker(ros::NodeHandle &nh);
    ~StandWorker();

};

StandWorker::StandWorker(ros::NodeHandle &nh) {
    this->nh_ = nh;
    this->angle_real_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle", 10
            );
    this->angle_gazebo_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle_gazebo", 1);

    /*! Init some param*/
    this->init_time_ = ros::Time::now().toSec();
    this->end_time_ = 0.0;
    this->angle_real_data_.data.resize(12);
    this->angle_gazebo_data.data.resize(12);
}

StandWorker::~StandWorker() {

}

void StandWorker::run(STATE_INTERIOR *cur_state) {
    ROS_INFO("Standing");
    this->cur_time_ = ros::Time::now().toSec();
    /*! delay for completing init*/
    if (this->cur_time_-this->init_time_>=5.0 && this->flag_ == 0){
        this->flag_ = 1;
    }
    /*! Stand in gazebo */
    if(this->flag_ == 1){
        for (int i = 0; i < 4; i++) {
            this->angle_gazebo_data.data[3*i] = 0.0;
            this->angle_gazebo_data.data[3*i+1] = this->init_hip_-this->command_hip_;
            this->angle_gazebo_data.data[3*i+2] = this->init_knee_+2*this->command_knee_;
        }
        if(this->command_hip_ < 60.0 ){
            this->command_hip_ = this->command_hip_+0.3;
        }
        if(this->command_knee_ < 57.5){
            this->command_knee_ = this->command_knee_ + 0.3;
        }
        if(this->command_hip_ >=60.0 && this->command_knee_>=57.5){
            this->flag_ = 2;
            this->end_time_ = ros::Time::now().toSec();
            /*! return the stand state */
            


        }
    }else if(this->flag_ == 0){
        for (int i = 0; i < 4; i++) {
            this->angle_gazebo_data.data[3*i] = 0.0;
            this->angle_gazebo_data.data[3*i+1] = this->init_hip_;
            this->angle_gazebo_data.data[3*i+2] = this->init_knee_;
        }
    }

    this->angle_gazebo_pub_.publish(this->angle_gazebo_data);
    this->angle_real_pub_.publish(this->angle_real_data_);

}

bool StandWorker::is_finished() {
    if((this->cur_time_-this->end_time_ >= 1.0) && this->flag_ == 2){
        ROS_INFO("Finish Stand State");
        return true;
    }else{
        return false;
    }

}

#endif
