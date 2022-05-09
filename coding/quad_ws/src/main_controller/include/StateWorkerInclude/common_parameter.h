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

    /*! Command Limit */
    const double JOY_CMD_VELX_MAX = 0.6;
    const double JOY_CMD_VELY_MAX = 0.3;
    const double JOY_CMD_YAW_MAX = 0.8;
    const double JOY_CMD_BODY_HEIGHT_VEL = 0.04;
    const double JOY_CMD_BODY_HEIGHT_MAX = 0.35;
    const double JOY_CMD_BODY_HEIGHT_MIN = 0.1;

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

    /*! constant */
    const double X_OFFSET = 0.17;
    const double Y_OFFSET = 0.15;
    const double HIP_LENGTH = 0.21;
    const double KNEE_LENGTH = 0.21;
    const int NUM_DOF = 12;
    const int NUM_LEG = 4;
    const double FOOT_SWING_CLEARANCE1 = 0.0f;
    const double FOOT_SWING_CLEARANCE2 = 0.4f;
    const double FOOT_DELTA_X_LIMIT = 0.1;
    const double FOOT_DELTA_Y_LIMIT = 0.1;

    /*! MPC */
    const int PLAN_HORIZON = 10;
    const int MPC_STATE_DIM = 13;
    const int MPC_CONSTRAINT_DIM = 20;

}


#endif 
