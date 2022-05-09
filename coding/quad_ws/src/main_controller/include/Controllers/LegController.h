//
// Created by demphi on 2022/3/22.
//

#ifndef _LEGCONTROLLER_
#define _LEGCONTROLLER_

#include "FootSwingTrajectory.h"
#include "MathTools/KinematicModel.h"
#include "MathTools/LegKinematics.h"
#include "common_parameter.h"

/*!
* Data returned from the legs to the control code.
 */
struct LegControllerData{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerData(){zero();}
    void zero(){
        p = Vec3::Zero();
        v = Vec3::Zero();
        J_temp = Mat22::Zero();
        J_temp_inv = Mat22::Zero();
        J = Mat33::Zero();
        J_inv = Mat33::Zero();
    }
    Vec3 p, v;
    Mat22 J_temp;
    Mat22 J_temp_inv;
    Mat33 J;
    Mat33 J_inv;

    /*! angle of three joints*/
    Vec3 q;
    Vec3 qd;
};

/*!
 * Data sent from the control algorithm to the legs.
 */
struct LegControllerCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerCommand() { zero(); }

    void zero(){
        qDes = Vec3::Zero();
        qdDes = Vec3::Zero();
        pDes = Vec3::Zero();
        vDes = Vec3::Zero();
    }
    Vec3  qDes, qdDes, pDes, vDes;
};


class LegController{
public:
    double dt_;
    LegControllerCommand leg_command_[4];
    LegControllerData leg_data_[4];

    KinematicModel* model_kinematic = new KinematicModel();
    LegKinematics* model_LegKinematic = new LegKinematics();

    std::vector<Eigen::VectorXd> rho_fix_list;
    std::vector<Eigen::VectorXd> rho_opt_list;

    LegController(){
        for (int i = 0; i < 4; i++) {
            this->leg_command_[i].zero();
            this->leg_data_[i].zero();
        }
        double leg_offset_x[4] = {};
        double leg_offset_y[4] = {};
        double motor_offset[4] = {};
        //0-FL  1-FR  2-RR  3-RL
        leg_offset_x[0] = 0.1805;
        leg_offset_x[1] = 0.1805;
        leg_offset_x[2] = -0.1805;
        leg_offset_x[3] = -0.1805;
        leg_offset_y[0] = 0.047;
        leg_offset_y[1] = -0.047;
        leg_offset_y[2] = 0.047;
        leg_offset_y[3] = -0.047;
        motor_offset[0] = 0.0838;
        motor_offset[1] = -0.0838;
        motor_offset[2] = 0.0838;
        motor_offset[3] = -0.0838;

        for (int i = 0; i < quad::NUM_LEG; i++) {
            Eigen::VectorXd rho_fix(5);
            rho_fix << leg_offset_x[i], leg_offset_y[i], motor_offset[i], quad::HIP_LENGTH, quad::KNEE_LENGTH;
            Eigen::VectorXd rho_opt(3);
            rho_opt << 0.0, 0.0, 0.0;
            rho_fix_list.push_back(rho_fix);
            rho_opt_list.push_back(rho_opt);
        }
    }
    ~LegController(){
        delete this->model_kinematic;
        delete this->model_LegKinematic;
    }

    void FirstUpdateData(STATE_INTERIOR *cur_state){
        this->dt_ = cur_state->plan_dt;
        for (int foot = 0; foot < 4; foot++) {
            this->leg_data_[foot].q(0) = 0.0;
            this->leg_data_[foot].q(1) = 30/quad::Rad2Deg;
            this->leg_data_[foot].q(2) = -60/quad::Rad2Deg;
            this->leg_data_[foot].p = this->model_LegKinematic->fk(
                    this->leg_data_[foot].q, rho_opt_list[foot], rho_fix_list[foot]);
            this->leg_data_[foot].J = this->model_LegKinematic->jac(
                    this->leg_data_[foot].q, rho_opt_list[foot], rho_fix_list[foot]);
            this->leg_data_[foot].J_inv = this->leg_data_[foot].J.inverse();
            this->leg_data_[foot].v = this->leg_data_[foot].J * this->leg_data_[foot].qd;

            /*! Update to state interior */
            cur_state->foot_p_robot.block<3, 1>(0, foot) = this->leg_data_[foot].p;
            cur_state->foot_v_robot.block<3, 1>(0, foot) = this->leg_data_[foot].v;
            cur_state->foot_q.block<3, 1>(0, foot) = this->leg_data_[foot].q;
            cur_state->foot_qd.block<3, 1>(0, foot) = this->leg_data_[foot].qd;
        }
    }

    void UpdateData() {
        for (int foot = 0; foot < 4; foot++) {
            this->leg_data_[foot].p = this->model_LegKinematic->fk(
                    this->leg_data_[foot].q, rho_opt_list[foot], rho_fix_list[foot]);
            this->leg_data_[foot].J = this->model_LegKinematic->jac(
                    this->leg_data_[foot].q, rho_opt_list[foot], rho_fix_list[foot]);
            this->leg_data_[foot].J_inv = this->leg_data_[foot].J.inverse();
            this->leg_data_[foot].v = this->leg_data_[foot].J * this->leg_data_[foot].qd;

        }
    }
    void UpdateCommand(){
            for (int foot = 0; foot < 4; foot++) {
                this->leg_command_[foot].qDes = this->model_kinematic->InverseKinematic(foot,
                                                                                        this->leg_command_[foot].pDes);
                Vec2 temp_v;
                temp_v.x() = this->leg_command_[foot].vDes.x();
                temp_v.y() = this->leg_command_[foot].vDes.z();
                Vec2 temp_ = this->leg_data_[foot].J_temp.inverse() * temp_v;
                this->leg_command_[foot].qdDes.x() = 0.0;
                this->leg_command_[foot].qdDes.y() = temp_.x();
                this->leg_command_[foot].qdDes.z() = temp_.y();
            }
        }

    void ComputeJacobian(Vec3 &q, Mat22 *J, Mat33 *J_3DoF, int leg) {
            double l1 = quad::HIP_LENGTH;
            double l2 = quad::KNEE_LENGTH;

            double s1 = std::sin(q(1));
            double s2 = std::sin(q(2));
            double c1 = std::cos(q(1));
            double c2 = std::cos(q(2));
            double s12 = s1 * c2 + c1 * s2;
            double c12 = c1 * c2 - s1 * s2;

            J->operator()(0, 0) = -(l1 * c1 + l2 * c12);
            J->operator()(0, 1) = -(l2 * c12);
            J->operator()(1, 0) = l1 * s1 + l2 * s12;
            J->operator()(1, 1) = l2 * s12;

            J_3DoF->operator()(0, 0) = 0;
            J_3DoF->operator()(0, 1) = -(l1 * c1 + l2 * c12);
            J_3DoF->operator()(0, 2) = -(l2 * c12);
            J_3DoF->operator()(1, 0) = l2 * c12 + l1 * c1;
            J_3DoF->operator()(1, 1) = 0;
            J_3DoF->operator()(1, 2) = 0;
            J_3DoF->operator()(2, 0) = 0;
            J_3DoF->operator()(2, 1) = l1 * s1 + l2 * s12;
            J_3DoF->operator()(2, 2) = l2 * s12;

        }

};



#endif 
