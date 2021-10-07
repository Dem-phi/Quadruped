//
// Created by demphi on 2021/10/1.
//

#ifndef _WALKWORKER_
#define _WALKWORKER_

#include "StateWorker.h"
#include "CAR.h"
#include "Hopf.h"
#include "GaitTable.h"

/**
 * @brief
 * @param
 */
class WalkWorker:public StateWorker{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_angle;
    std_msgs::Float64MultiArray msg_angle;

    CAR* model_CAR = new CAR(0.3, 0.02, 0.75);
    GaitTable* walk_table = new GaitTable();
    Eigen::Vector2f amplitude;

    virtual void run();
    virtual bool is_finished();

    WalkWorker(ros::NodeHandle &nh);
    ~WalkWorker();
};

WalkWorker::WalkWorker(ros::NodeHandle &nh) {
    this->nh = nh;
    this->pub_angle = this->nh.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle", 1);
    this->amplitude = model_CAR->Calculate_amplitude();
    //init
    this->msg_angle.data.resize(12);
}

WalkWorker::~WalkWorker() {
    delete this->model_CAR;
    delete this->walk_table;
}

void WalkWorker::run() {
    ROS_INFO("Walking");
//    cout << amplitude.x() << "   " << amplitude.y() << endl;
    msg_angle.data = walk_table->Get_Angle_From_Table().data;
    pub_angle.publish(msg_angle);
}

bool WalkWorker::is_finished() {

    return false;
}

#endif 
