//
// Created by shuoy on 11/1/21.
//

#ifndef A1_CPP_A1BASICEKF_H
#define A1_CPP_A1BASICEKF_H

#include "common_parameter.h"
#include "common_defination.h"
#include "MathTools//Utils.h"


// state estimator parameters
#define STATE_SIZE 18
#define MEAS_SIZE 28
#define PROCESS_NOISE_PIMU 0.01
#define PROCESS_NOISE_VIMU 0.01
#define PROCESS_NOISE_PFOOT 0.01
#define SENSOR_NOISE_PIMU_REL_FOOT 0.001
#define SENSOR_NOISE_VIMU_REL_FOOT 0.1
#define SENSOR_NOISE_ZFOOT 0.001

using namespace quad;

// implement a basic error state KF to estimate robot pose
// assume orientation is known from a IMU (state.root_rot_mat)
class A1BasicEKF {
public:
    A1BasicEKF (){
        // constructor
        eye3.setIdentity();
        // C is fixed
        C.setZero();
        for (int i=0; i<NUM_LEG; ++i) {
            C.block<3,3>(i*3,0) = -eye3;  //-pos
            C.block<3,3>(i*3,6+i*3) = eye3;  //foot pos
            C.block<3,3>(NUM_LEG*3+i*3,3) = eye3;  // vel
            C(NUM_LEG*6+i,6+i*3+2) = 1;  // height z of foot
        }

        // Q R are fixed
        Q.setIdentity();
        Q.block<3,3>(0,0) = PROCESS_NOISE_PIMU*eye3;               // position transition
        Q.block<3,3>(3,3) = PROCESS_NOISE_VIMU*eye3;               // velocity transition
        for (int i=0; i<NUM_LEG; ++i) {
            Q.block<3,3>(6+i*3,6+i*3) = PROCESS_NOISE_PFOOT*eye3;  // foot position transition
        }

        R.setIdentity();
        for (int i=0; i<NUM_LEG; ++i) {
            R.block<3,3>(i*3,i*3) = SENSOR_NOISE_PIMU_REL_FOOT*eye3;                        // fk estimation
            R.block<3,3>(NUM_LEG*3+i*3,NUM_LEG*3+i*3) = SENSOR_NOISE_VIMU_REL_FOOT*eye3;      // vel estimation
            R(NUM_LEG*6+i,NUM_LEG*6+i) = SENSOR_NOISE_ZFOOT;                               // height z estimation
        }

        // set A to identity
        A.setIdentity();

        // set B to zero
        B.setZero();

        assume_flat_ground = true;

    }

    A1BasicEKF (bool assume_flat_ground_){
        // constructor
        assume_flat_ground = assume_flat_ground_;
        // change R according to this flag, if we do not assume the robot moves on flat ground,
        // then we cannot infer height z using this way
        if (assume_flat_ground == false) {
            for (int i=0; i<NUM_LEG; ++i) {
                R(NUM_LEG*6+i,NUM_LEG*6+i) = 1e5;   // height z estimation not reliable
            }
        }
    }

    void init_state(STATE_INTERIOR& state){
        filter_initialized = true;
        P.setIdentity();
        P = P * 3;

        // set initial value of x
        x.setZero();
        x.segment<3>(0) = Eigen::Vector3d(0, 0, 0.09);
        for (int i = 0; i < NUM_LEG; ++i) {
            Eigen::Vector3d fk_pos = state.foot_p_abs.block<3, 1>(0, i);
            x.segment<3>(6 + i * 3) = state.rotate_matrix * fk_pos + x.segment<3>(0);
        }
    }

    void update_estimation(STATE_INTERIOR& state, double dt){
        // update A B using latest dt
        A.block<3, 3>(0, 3) = dt * eye3;
        B.block<3, 3>(3, 0) = dt * eye3;

        // control input u is Ra + ag
        Eigen::Vector3d u = state.rotate_matrix * state.b_acc + Eigen::Vector3d(0, 0, -9.81);

        // contact estimation, do something very simple first
        if (state.gait_type == quad::STAND) {  // stand
            for (int i = 0; i < NUM_LEG; ++i) estimated_contacts[i] = 1.0;
        } else {  // walk
            for (int i = 0; i < NUM_LEG; ++i) {
                estimated_contacts[i] = std::min(std::max((state.foot_force(i)) / (100.0 - 0.0), 0.0), 1.0);
//        estimated_contacts[i] = 1.0/(1.0+std::exp(-(state.foot_force(i)-100)));
            }
        }
        // update Q
        Q.block<3, 3>(0, 0) = PROCESS_NOISE_PIMU * dt / 20.0 * eye3;
        Q.block<3, 3>(3, 3) = PROCESS_NOISE_VIMU * dt * 9.8 / 20.0 * eye3;
        // update Q R for legs not in contact
        for (int i = 0; i < NUM_LEG; ++i) {
            Q.block<3, 3>(6 + i * 3, 6 + i * 3)
                    =
                    (1 + (1 - estimated_contacts[i]) * 1e3) * dt * PROCESS_NOISE_PFOOT * eye3;  // foot position transition
            // for estimated_contacts[i] == 1, Q = 0.002
            // for estimated_contacts[i] == 0, Q = 1001*Q

            R.block<3, 3>(i * 3, i * 3)
                    = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_PIMU_REL_FOOT *
                      eye3;                       // fk estimation
            R.block<3, 3>(NUM_LEG * 3 + i * 3, NUM_LEG * 3 + i * 3)
                    = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_VIMU_REL_FOOT * eye3;      // vel estimation
            if (assume_flat_ground) {
                R(NUM_LEG * 6 + i, NUM_LEG * 6 + i)
                        = (1 + (1 - estimated_contacts[i]) * 1e3) * SENSOR_NOISE_ZFOOT;       // height z estimation
            }
        }


        // process update
        xbar = A*x + B*u ;
        Pbar = A * P * A.transpose() + Q;

        // measurement construction
        yhat = C*xbar;
//    leg_v = (-J_rf*av-skew(omega)*p_rf);
//    r((i-1)*3+1:(i-1)*3+3) = body_v - R_er*leg_v;
        // actual measurement
        for (int i=0; i<NUM_LEG; ++i) {
            Eigen::Vector3d fk_pos = state.foot_p_abs.block<3,1>(0,i);
            y.block<3,1>(i*3,0) = state.rotate_matrix*fk_pos;   // fk estimation
            Eigen::Vector3d leg_v = -state.foot_v_robot.block<3,1>(0,i) - Utils::skew(state.b_angle_vel)*fk_pos;
            y.block<3,1>(NUM_LEG*3+i*3,0) =
                    (1.0-estimated_contacts[i])*x.segment<3>(3) +  estimated_contacts[i]*state.rotate_matrix*leg_v;      // vel estimation

            y(NUM_LEG*6+i) =
                    (1.0-estimated_contacts[i])*(x(2)+fk_pos(2)) + estimated_contacts[i]*0;                               // height z estimation
        }

        S = C * Pbar *C.transpose() + R;
        S = 0.5*(S+S.transpose());

        error_y = y - yhat;
        Serror_y = S.fullPivHouseholderQr().solve(error_y);

        x = xbar + Pbar * C.transpose() * Serror_y;

        SC = S.fullPivHouseholderQr().solve(C);
        P = Pbar - Pbar * C.transpose() * SC * Pbar;
        P = 0.5 * (P + P.transpose());

        // reduce position drift
        if (P.block<2, 2>(0, 0).determinant() > 1e-6) {
            P.block<2, 16>(0, 2).setZero();
            P.block<16, 2>(2, 0).setZero();
            P.block<2, 2>(0, 0) /= 10.0;
        }

        // final step
        // put estimated values back to A1CtrlStates& state
        for (int i = 0; i < NUM_LEG; ++i) {
            if (estimated_contacts[i] < 0.5) {
                state.estimate_contacts[i] = false;
            } else {
                state.estimate_contacts[i] = true;
            }
        }
//    std::cout << x.transpose() <<std::endl;
        state.estimate_position = x.segment<3>(0);
        state.estimate_vel = x.segment<3>(3);

        state.cur_position = x.segment<3>(0);
        state.cur_vel = x.segment<3>(3);
    }

    bool is_inited() {return filter_initialized;}

private:
    bool filter_initialized = false;
    // state
    // 0 1 2 pos 3 4 5 vel 6 7 8 foot pos FL 9 10 11 foot pos FR 12 13 14 foot pos RL 15 16 17 foot pos RR
    Eigen::Matrix<double, STATE_SIZE, 1> x; // estimation state
    Eigen::Matrix<double, STATE_SIZE, 1> xbar; // estimation state after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> P; // estimation state covariance
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Pbar; // estimation state covariance after process update
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> A; // estimation state transition
    Eigen::Matrix<double, STATE_SIZE, 3> B; // estimation state transition
    Eigen::Matrix<double, STATE_SIZE, STATE_SIZE> Q; // estimation state transition noise

    // observation
    // 0 1 2   FL pos residual
    // 3 4 5   FR pos residual
    // 6 7 8   RL pos residual
    // 9 10 11 RR pos residual
    // 12 13 14 vel residual from FL
    // 15 16 17 vel residual from FR
    // 18 19 20 vel residual from RL
    // 21 22 23 vel residual from RR
    // 24 25 26 27 foot height
    Eigen::Matrix<double, MEAS_SIZE, 1> y; //  observation
    Eigen::Matrix<double, MEAS_SIZE, 1> yhat; // estimated observation
    Eigen::Matrix<double, MEAS_SIZE, 1> error_y; // estimated observation
    Eigen::Matrix<double, MEAS_SIZE, 1> Serror_y; // S^-1*error_y
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> C; // estimation state observation
    Eigen::Matrix<double, MEAS_SIZE, STATE_SIZE> SC; // S^-1*C
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> R; // estimation state observation noise
    // helper matrices
    Eigen::Matrix<double, 3, 3> eye3; // 3x3 identity
    Eigen::Matrix<double, MEAS_SIZE, MEAS_SIZE> S; // Innovation (or pre-fit residual) covariance
    Eigen::Matrix<double, STATE_SIZE, MEAS_SIZE> K; // kalman gain


    bool assume_flat_ground = false;

    // variables to process foot force
    double smooth_foot_force[4];
    double estimated_contacts[4];
};


#endif //A1_CPP_A1BASICEKF_H
