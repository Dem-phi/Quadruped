//
// Created by demphi on 2021/10/26.
//
#include "Rotations.h"
#include "Common_Definition.h"
#include "OCEKF.h"
#include "Controller.h"
#include "sensor_msgs/Imu.h"


namespace LSE{


    /*! the time delay may be the problem*/
    Controller::Controller(ros::NodeHandle &nh){
        this->nh = nh;
        this->imu_sub_ = this->nh.subscribe("/imu", 1, &Controller::GetImuCallback, this);
        this->encoder_sub_ = this->nh.subscribe("/encoder", 1, &Controller::GetEncoderCallback, this);
    }

    void Controller::GetImuCallback(const sensor_msgs::ImuConstPtr &msg) {

    }

    void Controller::GetEncoderCallback(const sensor_msgs::ImuConstPtr &msg) {

    }

    Eigen::Matrix3d Controller::gamma(const int &k, const Eigen::Matrix3d &w, const double &dt) {
        


        return Eigen::Matrix3d();
    }

    int Controller::factorial(const int &k) {
        if(k == 1){
            return 1;
        }else{
            return k*factorial(k-1);
        }
    }


}