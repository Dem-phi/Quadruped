//
// Created by demphi on 2021/10/26.
//

#ifndef _OCEKF_H
#define _OCEKF_H

#include "eigen3/Eigen/Dense"
#include "Common_Definition.h"
#include "Controller.h"

namespace LSE{

class OCEKF{
public:
    /* -------------------- Constructor/Destructor --------------------- */
    /*! Constructor
     * @param
     */
    OCEKF(Controller* pController);

    /*! Destructor */
    ~OCEKF();

    /* -------------------- Filter handling --------------------- */
    void predict();

    void update(const double& t);

    void reset();

    /*! Loads overall parameters*/
    void loadParam();

private:


    /*! Structure of filter intern state */
    struct InternState{
        /*! eigen 对齐指针    */
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*! Time of estimate */
        double t_;
        /*! Position estimate */
        Eigen::Vector3d r_;
        /*! Velocity estimate */
        Eigen::Vector3d v_;
        /*! Attitude estimate (quaternion) */
        Eigen::Quaternion<double> q_;
        /*! Rotational rate estimate (bias corrected) */
        Eigen::Vector3d w_;
        /*! Foothold estimate */
        Eigen::Matrix<double,3,LSE_N_LEG> p_;
        /*! Contact flag counter */
        //CF CFC_;
        /*! Estimate of accelerometer bias */
        Eigen::Vector3d bf_;
        /*! Estimate of gyroscope bias */
        Eigen::Vector3d bw_;
        /*! Estimate of covariance matrix */
        Eigen::Matrix<double,15+3*LSE_N_LEG,15+3*LSE_N_LEG> P_;
        /*! Last position estimate */
        Eigen::Vector3d rLast_;
        /*! Last velocity estimate */
        Eigen::Vector3d vLast_;
        /*! Last attitude estimate */
        Eigen::Quaternion<double> qLast_;
        /*! Linearization point for footholds */
        Eigen::Matrix<double,3,LSE_N_LEG> pLin_;
        /*! Linearization point for rotational rate */
        Eigen::Vector3d wLin_;
        /*! Linearization point for accelerometer measurement for position Jacobian */
        Eigen::Vector3d f1Lin_;
        /*! Linearization point for accelerometer measurement for velocity Jacobian */
        Eigen::Vector3d f2Lin_;
        /*! Current corrected accelerometer measurement */
        Eigen::Vector3d f_;
        /*! Time of last update */
        double tLast_;
    };

    /* -------------------- Filtering/Predicting/Updating --------------------- */
    /*! Filters the referenced internal state up to the given time
     * @param[in/out]	x	Filter state to be filtered
     * @param[in]		t	Desired filter time
     */
    void filterState(InternState& x,const double& t);

    /*! Predicts the referenced internal state using the given IMU measurement
     * @param[in/out]	x	Filter state to be filtered
     * @param[in]		t	Desired filter time
     * @param[in]		m	IMU measurement
     */
    void predictState(InternState& x,const double& t, const _imu_meas_& m);

    /*! Updates the referenced internal state using the given Encoder measurement
     * @param[in/out]	x	Filter state to be filtered
     * @param[in]		m	Encoder measurement
     */
    void updateState(InternState& x,const _encoder_meas_& m);



    /* -------------------- Pointers and filter states --------------------- */
    /*! Pointer to main class Manager */
    Controller* pController_;
    /*! Safe state (where the chance is high that all measurements have arrived) */
    InternState xs_;
    /*! Predicted state */
    InternState xp_;




};

}

#endif 
