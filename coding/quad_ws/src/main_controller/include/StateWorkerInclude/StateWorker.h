//
// Created by demphi on 2021/10/1.
//

#ifndef _STATEWORKER_
#define _STATEWORKER_

#include "ros/ros.h"
#include "CAR.h"
#include "Hopf.h"
#include "GaitTable.h"
#include "GaitTransition.h"
#include "KinematicModel.h"
#include "Controllers/GaitScheduler.h"
#include "Controllers/FootSwingTrajectory.h"
#include "Controllers/LegController.h"


class StateWorker{
public:
    ros::NodeHandle nh;
    virtual void run() = 0;
    virtual bool is_finished() = 0;
    bool is_working = 0;

    StateWorker(){};
    ~StateWorker(){};
};

#endif
