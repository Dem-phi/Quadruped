//
// Created by demphi on 2021/10/1.
//

#ifndef _STANDWORKER_
#define _STANDWORKER_

#include "StateWorker.h"
#include "geometry_msgs/Vector3.h"

/**
 * @brief send angle msg and stand
 * @param none
 */
class StandWorker: public StateWorker{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_angle;

    //just control one leg
    geometry_msgs::Vector3 LF_angle;

    virtual void run();
    virtual bool is_finished();

    StandWorker(ros::NodeHandle &nh);
    ~StandWorker();

};

StandWorker::StandWorker(ros::NodeHandle &nh) {
    this->nh = nh;
    this->pub_angle = nh.advertise<geometry_msgs::Vector3>(
            "/quad/set_angle", 10
            );
    this->LF_angle.x = 10;
    this->LF_angle.y = 10;
    this->LF_angle.z = 0;
}

StandWorker::~StandWorker() {

}

void StandWorker::run() {
    ROS_INFO("Stand");
    pub_angle.publish(LF_angle);
}

bool StandWorker::is_finished() {
    return false;
}

#endif
