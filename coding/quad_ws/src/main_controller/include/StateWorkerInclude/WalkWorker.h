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
public:
    ros::NodeHandle nh;
    ros::Publisher pub_angle;
    std_msgs::Float64MultiArray msg_angle;

    CAR* model_CAR = new CAR(0.3, 0.8, 0.02, quad::WALK_BETA);
    GaitTable* walk_table = new GaitTable();
    Hopf* model_Hopf = new Hopf({7.4, 5.3}, quad::WALK_BETA, quad::WALK_PHI);
    Eigen::Vector2f amplitude;

    virtual void run();
    virtual bool is_finished();

    WalkWorker(ros::NodeHandle &nh, float velocity);
    ~WalkWorker();
};

WalkWorker::WalkWorker(ros::NodeHandle &nh, float velocity) {
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
    delete this->model_Hopf;
}

void WalkWorker::run() {
    ROS_INFO("Walking");
    //cout << amplitude.x() << "   " << amplitude.y() << endl;
    this->msg_angle = model_Hopf->CalculateAngle(this->msg_angle);
//    msg_angle.data = walk_table->Get_Angle_From_Table().data;
    pub_angle.publish(msg_angle);
}

bool WalkWorker::is_finished() {

    return false;
}

#endif 
