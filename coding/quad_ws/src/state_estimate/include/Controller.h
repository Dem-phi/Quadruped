//
// Created by demphi on 2021/10/27.
//

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "map"
#include "ros/ros.h"

namespace LSE{
    class OCEKF;

    /*!
     update the IMU and encoder data; use OCEKF for estimate*/
    class Controller{
    public:
        ros::NodeHandle nh;
        ros::Subscriber imu_sub_, encoder_sub_;

        /* -------------------- Callback Function --------------------- */
        /*! Callback function, Searches for the first measurement after time t
         * @return const pointer to measurement
         * @param[in/out]	t	time of measurement, changed to precise measurement time
         */
        //const _imu_meas_* GetImuCallback(double& t);
        void GetImuCallback(const sensor_msgs::ImuConstPtr &msg);
        /*! Callback function, Searches for the first measurement after time t
         * @return const pointer to measurement
         * @param[in/out]	t	time of measurement, changed to precise measurement time
         */
        //const _encoder_meas_* getEncMeas(double& t);
        void GetEncoderCallback(const sensor_msgs::ImuConstPtr &msg);

        Controller(ros::NodeHandle &nh);
        ~Controller();


        friend class OCEKF;


    private:
        OCEKF* pointOCEKF;

        /*! Returns gamma 3x3 matrix
         * @return	gamma matrix
         * @param[in]	k	Order
         * @param[in]	w	Rotational rate
         * @param[in]	dt	Time difference
         */
        Eigen::Matrix3d gamma(const int& k, const Eigen::Matrix3d& w, const double& dt);

        /*! Factorial
         * @return	factorial of k
         * @param[in]	k	Natural number
         */
        int factorial(const int& k);
        /* -------------------- Measurement Storage --------------------- */
        /*! Map storage of Imu Measurements */
        std::map<double,_imu_meas_> Imu_Meas_List;
        /*! Map storage of encoder Measurements */
        std::map<double,_encoder_meas_> Encoder_Meas_List;

        /* -------------------- Parameters --------------------- */
        /*! Gravity vector in world coordinate frame */
        const Eigen::Vector3d g_;
        /*! Imu time offset, real time = timestamp + tImu_ */
        double tImu_;
        /*! Encoder time offset, real time = timestamp + tEnc_ */
        double tEnc_;

    };
}


#endif 
