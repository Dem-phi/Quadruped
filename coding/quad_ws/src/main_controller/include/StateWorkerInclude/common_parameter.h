//
// Created by demphi on 2021/10/1.
//

#ifndef _COMMON_PARAMETER_
#define _COMMON_PARAMETER_

#include "reference/common_include.h"

/**
  * @param beta -> duty cycle
  * @param phi  -> phase position of Light Hind
  */
namespace quad{
    // rad to deg -> deg = rad * Rad2Deg
    // deg to rad -> rad = deg / Rad2Deg
    const double Rad2Deg = 45.0/ atan(1);

    const double SAFETY_TOLERANCE = 10.0;
    /*! offset of gazebo*/
    const double DEFAULT_SHOULD_ANGLE = 10.0;
    const double HIP_ANGLE_OFFSET = 45.0;
    const double KNEE_ANGLE_OFFSET = 0.0;

    /*! bias of gazebo, use degree*/
    const Eigen::Vector3f GAZEBO_BIAS = {0.0, 30.0, -60.0};

    /*! Gait Scheduler */
    const float WALK_BETA = 0.75;
    const float TROT_BETA = 0.5;
    const float PACE_BETA = 0.5;
    const float GALLOP_BETA = 0.5;
    const Eigen::Vector4f WALK_PHI = {0, 0.5, 0.25, 0.75};
    const Eigen::Vector4f TROT_PHI = {0, 0.5, 0, 0.5};
    const Eigen::Vector4f PACE_PHI = {0, 0.5, 0.5, 0};
    const Eigen::Vector4f GALLOP_PHI = {0, 0, 0.5, 0.5};

    /*! dog constant */
    const double X_OFFSET = 0.3;
    const double Y_OFFSET = 0.2;
    const double HIP_LENGTH = 0.4;
    const double KNEE_LENGTH = 0.4;
    const int NUM_DOF = 12;
    const int NUM_LEG = 4;

    /*! MPC */
    const int PLAN_HORIZON = 10;
    const int MPC_STATE_DIM = 13;
    const int MPC_CONSTRAINT_DIM = 20;

}


#endif 
