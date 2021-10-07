//
// Created by demphi on 2021/10/1.
//

#ifndef _FINITESTATEMACHINE_
#define _FINITESTATEMACHINE_

#include "stdarg.h"
#include "Workers_common_include.h"

class FSM{
private:
    // main loop function runs in a specific frequency
    ros::Timer FSM_Timer;

public:
    ros::NodeHandle nh;
    vector<StateWorker*> Workers;
    int flow = 0;

    FSM(ros::NodeHandle &nh);
    ~FSM();
    void loop(const ros::TimerEvent &);
    void set_timer();
    void build_ScheduleTable(int Schedule, ...);


};

FSM::FSM(ros::NodeHandle &nh) {
    this->nh = nh;
}

FSM::~FSM() {
    // release
    for(auto each:this->Workers){
        delete each;
    }
}

void FSM::loop(const ros::TimerEvent &) {
    //running the schedule table
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
    this->FSM_Timer = nh.createTimer(ros::Duration(0.01), &FSM::loop, this);
}

void FSM::build_ScheduleTable(int Schedule, ...) {
    va_list arg_ptr;
    va_start(arg_ptr, Schedule);
    while(Schedule != quad::END){
        switch (Schedule) {
            case quad::STAND:{
                StandWorker* tmp_Worker = new StandWorker(this->nh);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::WALK:{
                float velocity = va_arg(arg_ptr, double);
                WalkWorker* tmp_Worker = new WalkWorker(this->nh);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::TROT:{
                TrotWorker* tmp_Worker = new TrotWorker(this->nh);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::PACE:{
                PaceWorker* tmp_Worker = new PaceWorker(this->nh);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::GALLOP:{
                GallopWorker* tmp_Worker = new GallopWorker(this->nh);
                this->Workers.push_back((StateWorker* )tmp_Worker);
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


#endif
