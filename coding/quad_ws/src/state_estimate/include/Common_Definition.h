//
// Created by demphi on 2021/10/26.
//

#ifndef _COMMON_DEFINITION_
#define _COMMON_DEFINITION_

#define LSE_DOF_LEG 3
#define LSE_N_LEG 4

#include "sensor_msgs/Imu.h"

namespace LSE{

    /*! Encoder measurement structure */
    struct _encoder_meas_{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*! Encoder measurement */
        Eigen::Matrix<double,LSE_DOF_LEG,LSE_N_LEG> e_;
    };

    /*! IMU measurement structure */
    struct _imu_meas_{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*! Accelerometer measurement */
        Eigen::Vector3d f_;
        /*! Gyroscope measurement */
        Eigen::Vector3d w_;
    };

    /*! State of robot main body */
     struct _state_info_{
         /*! align the memory */
         EIGEN_MAKE_ALIGNED_OPERATOR_NEW
         /*! Time */
         double t_;
         /*! Position */
         Eigen::Vector3d r_;
         /*! Velocity */
         Eigen::Vector3d v_;
         /*! Attitude */
         Eigen::Quaternion<double> q_;
         /*! Rotational rate */
         Eigen::Vector3d w_;
         /*! Covariance matrix*/
         Eigen::Matrix<double,12,12> P_;
     };


}

#endif 
