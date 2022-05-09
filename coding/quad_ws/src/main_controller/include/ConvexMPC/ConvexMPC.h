//
// Created by demphi on 2022/4/26.
//

#ifndef MAIN_CONTROLLER_CONVEXMPC_H
#define MAIN_CONTROLLER_CONVEXMPC_H

#include "MathTools/Utils.h"
#include "OsqpEigen/OsqpEigen.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Eigenvalues"
#include "eigen3/Eigen/Sparse"

using namespace quad;

class ConvexMPC{
public:
    /*! Weight Matrix*/
    Eigen::Matrix<double, MPC_STATE_DIM*PLAN_HORIZON, 1> q_weights_mpc;
    Eigen::Matrix<double, NUM_DOF*PLAN_HORIZON, 1> r_weights_mpc;
    //Eigen::MatrixXd Q;
    //Eigen::MatrixXd R;
    Eigen::DiagonalMatrix<double, MPC_STATE_DIM * PLAN_HORIZON> Q;
    Eigen::DiagonalMatrix<double, NUM_DOF * PLAN_HORIZON> R;
    Eigen::SparseMatrix<double> Q_sparse;
    Eigen::SparseMatrix<double> R_sparse;

    /*! Continue Time Dynamics */
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_c;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_c;
    Eigen::Matrix<double, MPC_STATE_DIM + NUM_DOF, MPC_STATE_DIM + NUM_DOF> AB_mat_c;

    /*! Discrete Time Dynamics*/
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> A_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM, NUM_DOF> B_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM + NUM_DOF, MPC_STATE_DIM + NUM_DOF> AB_mat_d;
    Eigen::Matrix<double, MPC_STATE_DIM * PLAN_HORIZON, NUM_DOF> B_mat_d_list;

    /*! QP Formulation */
    Eigen::Matrix<double, MPC_STATE_DIM*PLAN_HORIZON, MPC_STATE_DIM> A_qp;
    Eigen::Matrix<double, MPC_STATE_DIM*PLAN_HORIZON, NUM_DOF*PLAN_HORIZON> B_qp;

    /*! Minimize 1/2*X'*p*X+X'*q
     * s.t. lb <= Ac*x <= ub
     * */
    Eigen::SparseMatrix<double> hessian; // p
    Eigen::SparseMatrix<double> linear_constraints; // Ac
    Eigen::Matrix<double, NUM_DOF*PLAN_HORIZON, 1> gradient; //q
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM*PLAN_HORIZON, 1> lb;
    Eigen::Matrix<double, MPC_CONSTRAINT_DIM*PLAN_HORIZON, 1> ub;

    ConvexMPC(Eigen::VectorXd &_q_weights, Eigen::VectorXd &_r_weights){
        this->mu = 0.3;
        //reserve size for sparse matrix
        this->Q_sparse = Eigen::SparseMatrix<double>(MPC_STATE_DIM*PLAN_HORIZON,
                                                     MPC_STATE_DIM*PLAN_HORIZON);
        this->R_sparse = Eigen::SparseMatrix<double>(NUM_DOF*PLAN_HORIZON,
                                                     NUM_DOF*PLAN_HORIZON);
        this->q_weights_mpc.resize(MPC_STATE_DIM*PLAN_HORIZON);
        this->r_weights_mpc.resize(NUM_DOF*PLAN_HORIZON);

        // init Q
        for (int i = 0; i < PLAN_HORIZON; i++) {
            this->q_weights_mpc.segment(i*MPC_STATE_DIM, MPC_STATE_DIM) = _q_weights;
        }
        /*! why should multiply 2 ?*/
        this->Q.diagonal() = 2*this->q_weights_mpc;
        for (int i = 0; i < MPC_STATE_DIM*PLAN_HORIZON ; i++) {
            this->Q_sparse.insert(i, i) = 2*this->q_weights_mpc(i);
        }

        // init R
        for (int i = 0; i < PLAN_HORIZON; i++) {
            this->r_weights_mpc.segment(i*NUM_DOF, NUM_DOF) = _r_weights;
        }
        this->R.diagonal() = 2*this->r_weights_mpc;
        for (int i = 0; i < PLAN_HORIZON*NUM_DOF; i++) {
            this->R_sparse.insert(i, i) = 2*this->r_weights_mpc(i);
        }

        this->linear_constraints.resize(MPC_CONSTRAINT_DIM*PLAN_HORIZON, NUM_DOF*PLAN_HORIZON);
        //init linear constraints
        for (int i = 0; i < NUM_LEG * PLAN_HORIZON; i++) {
            this->linear_constraints.insert(0+5*i, 0+3*i) = 1;
            this->linear_constraints.insert(1+5*i, 0+3*i) = 1;
            this->linear_constraints.insert(2+5*i, 1+3*i) = 1;
            this->linear_constraints.insert(3+5*i, 1+3*i) = 1;
            this->linear_constraints.insert(4+5*i, 2+3*i) = 1;

            this->linear_constraints.insert(0+5*i, 2+3*i) = this->mu;
            this->linear_constraints.insert(1+5*i, 2+3*i) = this->mu*(-1);
            this->linear_constraints.insert(2+5*i, 2+3*i) = this->mu;
            this->linear_constraints.insert(3+5*i, 2+3*i) = this->mu*(-1);

        }

    }

    ~ConvexMPC(){

    }

    /*! Init and Reset Matrix to Zero*/
    void Reset(){
        A_mat_c.setZero();
        B_mat_c.setZero();
        AB_mat_c.setZero();

        A_mat_d.setZero();
        B_mat_d.setZero();
        AB_mat_d.setZero();
        B_mat_d_list.setZero();

        A_qp.setZero();
        B_qp.setZero();
        gradient.setZero();
        lb.setZero();
        ub.setZero();
    }

    /*! Update A_mat_c*/
    void Calculate_A_mat_c(Eigen::Vector3d _rpy){
        double cos_yaw = cos(_rpy.z());
        double sin_yaw = sin(_rpy.z());

        Mat33 angle_vel_to_rpy_rate;
        angle_vel_to_rpy_rate <<  cos_yaw, sin_yaw, 0,
                                 -sin_yaw, cos_yaw, 0,
                                  0      , 0      , 1;

        this->A_mat_c.block<3, 3>(0,6) = angle_vel_to_rpy_rate;
        this->A_mat_c.block<3, 3>(3, 9) = Mat33::Identity();
        this->A_mat_c(11, NUM_DOF) = 1;
    }

    /*! Update B_mat_c*/
    void Calculate_B_mat_c(double _robot_mass, const Mat33 &_inertia_matrix, Mat33 _rotate_matrix,
                           Eigen::Matrix<double, 3, NUM_LEG> _foot_position){
        Mat33 inertia_matrix_world;
        inertia_matrix_world = _rotate_matrix * _inertia_matrix * _rotate_matrix.transpose();
        for (int i = 0; i < NUM_LEG; i++) {
            this->B_mat_c.block<3, 3>(6, 3*i) =
                    inertia_matrix_world.inverse() * Utils::skew(_foot_position.block<3, 1>(0, i));
            this->B_mat_c.block<3, 3>(9, 3*i) =
                    (1/_robot_mass)*Eigen::Matrix3d::Identity();
        }

    }

    /*! State Space Discrete */
    void State_space_discrete(double _dt){
        this->A_mat_d = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() +
                        this->A_mat_c * _dt;
        this->B_mat_d = this->B_mat_c * _dt;
    }

    /*! Update qp matrix*/
    void Calculate_qp(STATE_INTERIOR &state){
        // calculate A_qp and B_qp
        for (int i = 0; i < PLAN_HORIZON; ++i) {
            if (i == 0) {
                A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) = A_mat_d;
            }
            else {
                A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * i, 0) =
                        A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i-1), 0)*A_mat_d;
            }
            for (int j = 0; j < i + 1; ++j) {
                if (i-j == 0) {
                    B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) =
                            B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
                } else {
                    B_qp.block<MPC_STATE_DIM, NUM_DOF>(MPC_STATE_DIM * i, NUM_DOF * j) =
                            A_qp.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (i-j-1), 0)
                            * B_mat_d_list.block<MPC_STATE_DIM, NUM_DOF>(j * MPC_STATE_DIM, 0);
                }
            }
        }

        // transform to QP Problems
        // Calculate Hessian
        Eigen::Matrix<double, NUM_DOF*PLAN_HORIZON, NUM_DOF*PLAN_HORIZON> dense_hessian;
        dense_hessian = (this->B_qp.transpose() * this->Q * this->B_qp);
        dense_hessian += this->R;
        this->hessian = dense_hessian.sparseView();

        // calculate gradient
        Eigen::Matrix<double, 13*PLAN_HORIZON, 1> tmp_vec = this->A_qp* state.mpc_states;
        tmp_vec -= state.mpc_states_list;
        this->gradient = this->B_qp.transpose() * this->Q * tmp_vec;

        this->fz_min = 0;
        this->fz_max = 180;

        // Calculate lower bound and upper bound
        Eigen::VectorXd lb_one_horizon(MPC_CONSTRAINT_DIM);
        Eigen::VectorXd ub_one_horizon(MPC_CONSTRAINT_DIM);
        for (int i = 0; i < NUM_LEG; i++) {
            lb_one_horizon.segment<5>(i*5) << 0, -OsqpEigen::INFTY, 0, -OsqpEigen::INFTY, fz_min * state.contacts[i];
            ub_one_horizon.segment<5>(i*5) << OsqpEigen::INFTY, 0, OsqpEigen::INFTY, 0, fz_max * state.contacts[i];
        }
        for (int i = 0; i < PLAN_HORIZON; i++) {
            lb.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = lb_one_horizon;
            ub.segment<MPC_CONSTRAINT_DIM>(i * MPC_CONSTRAINT_DIM) = ub_one_horizon;
        }

    }
private:
    /*! constraint of force */
    double fz_min = 0;
    double fz_max = 180;

    double mu = 0.3;

};

#endif 
