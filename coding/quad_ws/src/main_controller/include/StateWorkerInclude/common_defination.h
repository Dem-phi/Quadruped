//
// Created by demphi on 2021/10/1.
//

#ifndef _COMMON_DEFINATION_
#define _COMMON_DEFINATION_

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose.h"
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
        /*! current state */
        geometry_msgs::Pose cur_state;

        /*! feedback info */
        std_msgs::Float64MultiArray position_feedback_info;
        std_msgs::Float64MultiArray velocity_feedback_info;
        std_msgs::Float64MultiArray current_feedback_info;
    };

}


#endif 
