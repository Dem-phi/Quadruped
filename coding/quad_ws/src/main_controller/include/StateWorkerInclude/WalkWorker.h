//
// Created by demphi on 2021/10/1.
//

#ifndef _WALKWORKER_
#define _WALKWORKER_

#include "StateWorker.h"

/**
 * @brief Walk state
 * @param input none
 */
class WalkWorker:public StateWorker{
private:
    ros::Publisher angle_gazebo_pub_, angle_real_pub_;

public:
    ros::NodeHandle nh_;

    std_msgs::Float64MultiArray angle_gazebo_data_;

    std_msgs::Float64MultiArray angle_real_data_;


    CAR* model_CAR = new CAR(0.3, 0.8, 0.02, quad::WALK_BETA);
    GaitTable* walk_table = new GaitTable();
    Hopf* model_Hopf = new Hopf(model_CAR->Calculate_amplitude(), quad::WALK_BETA, quad::WALK_PHI);
    Eigen::Vector2f amplitude;

    virtual void run(STATE_INTERIOR *cur_state);
    virtual bool is_finished();

    WalkWorker(ros::NodeHandle &nh, float velocity);
    ~WalkWorker();
};

WalkWorker::WalkWorker(ros::NodeHandle &nh, float velocity) {
    this->nh_ = nh;
    this->angle_gazebo_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle_gazebo", 1);
    this->angle_real_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle", 1);
    this->amplitude = model_CAR->Calculate_amplitude();

    /*! Init some param*/
    this->angle_gazebo_data_.data.resize(12);
    this->angle_real_data_.data.resize(3);

}

WalkWorker::~WalkWorker() {
    delete this->model_CAR;
    delete this->walk_table;
    delete this->model_Hopf;
}

void WalkWorker::run(STATE_INTERIOR *cur_state) {
    ROS_INFO("Walking");
    std::cout << this->amplitude << std::endl;
    /*! LF should->hip->knee (0, 1, 2)*/
    this->angle_gazebo_data_ = model_Hopf->CalculateAngle(this->angle_gazebo_data_);
    /*! pub to low computer, already send the degree*/
/*    this->angle_real_data_.data[0] = 0;
    this->angle_real_data_.data[1] = this->angle_gazebo_data_.data[1];
    this->angle_real_data_.data[2] = this->angle_gazebo_data_.data[2];*/
    /*! add some bias, remap to the gazebo, */
    for (int i = 0; i < 4; i++) {
        this->angle_gazebo_data_.data[3*i] = 0.0;
        this->angle_gazebo_data_.data[3*i+1] += quad::GAZEBO_BIAS.y();
        this->angle_gazebo_data_.data[3*i+2] += quad::GAZEBO_BIAS.z();

    }
/*    std::cout << this->angle_gazebo_data_ << std::endl;*/

    this->angle_real_pub_.publish(this->angle_real_data_);
    /*! pub to gazebo */
    this->angle_gazebo_pub_.publish(this->angle_gazebo_data_);
}

bool WalkWorker::is_finished() {

    return false;
}

#endif 
