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

    ros::Subscriber motor_sub_;

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


};

FSM::FSM(ros::NodeHandle &nh) {
    this->nh_ = nh;
    this->motor_sub_ = this->nh_.subscribe("/quad/motor_info", 1000, &FSM::MotorCallback ,this);

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
        this->Workers[this->flow]->run();
    }
}

void FSM::set_timer() {
    this->FSM_Timer_ = this->nh_.createTimer(ros::Duration(0.02), &FSM::loop, this);
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


#endif
