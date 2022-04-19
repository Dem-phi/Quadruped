//
// Created by demphi on 2021/10/1.
//

#ifndef _FINITESTATEMACHINE_
#define _FINITESTATEMACHINE_

#include "stdarg.h"
#include "Workers_common_include.h"

class FSM{
private:
    /*! main loop function runs in a specific frequency */
    ros::Timer FSM_Timer_;

    ros::Subscriber motor_sub_, imu_sub_, joy_sub_, EKF_sub_;

public:
    ros::NodeHandle nh_;
    std::vector<StateWorker*> Workers;
    int flow = 0;

    quad::STATE_INFO state_info_;

    FSM(ros::NodeHandle &nh);
    ~FSM();
    void loop(const ros::TimerEvent &);
    void set_timer();
    void build_ScheduleTable(int Schedule, ...);

    /*! Callback Function */
    void MotorCallback(const std_msgs::Float64MultiArray &msg);
    void IMUCallback(const sensor_msgs::Imu &msg);
    void EKFCallback(const std_msgs::Float64MultiArray &msg);

};

FSM::FSM(ros::NodeHandle &nh) {
    this->nh_ = nh;
    this->motor_sub_ = this->nh_.subscribe("/quad/motor_info", 1000, &FSM::MotorCallback ,this);
    this->imu_sub_ = this->nh_.subscribe("/body_imu", 1000, &FSM::IMUCallback, this);
    this->EKF_sub_ = this->nh_.subscribe("/quad/EKF_result", 1000, &FSM::EKFCallback, this);

    /*! Init some param */
    this->state_info_.position_feedback_info.data.resize(12);
    this->state_info_.velocity_feedback_info.data.resize(12);
    this->state_info_.current_feedback_info.data.resize(12);
}

FSM::~FSM() {
    // release
    for(auto each:this->Workers){
        delete each;
    }
}

void FSM::loop(const ros::TimerEvent &) {
    /*! running the schedule table */
    if(this->Workers[this->flow]->is_finished()){
        this->flow++;
        if(this->flow == this->Workers.size()){
            ROS_INFO("Finish ScheduleTable");
            exit(0);
        }
    }
    else{
        this->Workers[this->flow]->run(this->state_info_);
    }
}

void FSM::set_timer() {
    this->FSM_Timer_ = this->nh_.createTimer(ros::Duration(0.01), &FSM::loop, this);
}

void FSM::build_ScheduleTable(int Schedule, ...) {
    va_list arg_ptr;
    va_start(arg_ptr, Schedule);
    while(Schedule != quad::END){
        switch (Schedule) {
            case quad::STAND:{
                StandWorker* tmp_Worker = new StandWorker(this->nh_);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::WALK:{
                float velocity = va_arg(arg_ptr, double);
                WalkWorker* tmp_Worker = new WalkWorker(this->nh_, velocity);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::TROT:{
                float velocity = va_arg(arg_ptr, double);
                TrotWorker* tmp_Worker = new TrotWorker(this->nh_);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::PACE:{
                float velocity = va_arg(arg_ptr, double);
                PaceWorker* tmp_Worker = new PaceWorker(this->nh_);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::GALLOP:{
                float velocity = va_arg(arg_ptr, double);
                GallopWorker* tmp_Worker = new GallopWorker(this->nh_);
                this->Workers.push_back((StateWorker* )tmp_Worker);
                break;
            }
            default:
                ROS_ERROR("Wrong type of Schedule Table");
                exit(0);
                break;
        }
        Schedule = va_arg(arg_ptr, int);
    }
    return;
}

void FSM::MotorCallback(const std_msgs::Float64MultiArray &msg) {

}

void FSM::IMUCallback(const sensor_msgs::Imu &msg) {
    this->state_info_.cur_state.orientation = msg.orientation;
    this->state_info_.b_twist.angular = msg.angular_velocity;
    this->state_info_.b_linear_acceleration = msg.linear_acceleration;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(this->state_info_.cur_state.orientation, quat);
    Eigen::Quaterniond temp_quat;
    temp_quat.x() = this->state_info_.cur_state.orientation.x;
    temp_quat.y() = this->state_info_.cur_state.orientation.y;
    temp_quat.z() = this->state_info_.cur_state.orientation.z;
    temp_quat.w() = this->state_info_.cur_state.orientation.w;

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    this->state_info_.rpy_angle.x = roll;
    this->state_info_.rpy_angle.y = pitch;
    this->state_info_.rpy_angle.z = yaw;

}

void FSM::EKFCallback(const std_msgs::Float64MultiArray &msg){
    this->state_info_.cur_state.position.x = msg.data[0];
    this->state_info_.cur_state.position.y = msg.data[1];
    this->state_info_.cur_state.position.z = msg.data[2];

    this->state_info_.w_twist.linear.x = msg.data[3];
    this->state_info_.w_twist.linear.y = msg.data[4];
    this->state_info_.w_twist.linear.z = msg.data[5];

    this->state_info_.b_twist.linear.x = msg.data[6];
    this->state_info_.b_twist.linear.y = msg.data[7];
    this->state_info_.b_twist.linear.z = msg.data[8];
}


#endif
