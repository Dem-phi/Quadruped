//
// Created by demphi on 2021/10/1.
//

#ifndef _COMMON_DEFINATION_
#define _COMMON_DEFINATION_

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"


namespace quad{
    /*! State Mode */
    enum STATE_TYPE{
        INIT,
        STAND,
        WALK,
        TROT,
        PACE,
        GALLOP,
        END
    };

    /*! state info */
    struct STATE_INFO{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*! current state */
        geometry_msgs::Pose cur_state;

        /*! imu msg */
        geometry_msgs::Twist b_twist;
        geometry_msgs::Twist w_twist;
        geometry_msgs::Vector3 b_linear_acceleration;
        geometry_msgs::Vector3 rpy_angle;


        /*! feedback info */
        std_msgs::Float64MultiArray position_feedback_info;
        std_msgs::Float64MultiArray velocity_feedback_info;
        std_msgs::Float64MultiArray current_feedback_info;
    };

}


#endif 
