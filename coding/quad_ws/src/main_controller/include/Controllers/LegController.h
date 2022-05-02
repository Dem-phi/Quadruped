//
// Created by demphi on 2022/3/22.
//

#ifndef _LEGCONTROLLER_
#define _LEGCONTROLLER_

#include "FootSwingTrajectory.h"
#include "MathTools/KinematicModel.h"
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

    LegController(){
        for (int i = 0; i < 4; i++) {
            this->leg_command_[i].zero();
            this->leg_data_[i].zero();
        }
    }
    ~LegController(){
        delete this->model_kinematic;
    }

    void FirstUpdateData(STATE_INTERIOR *cur_state){
        this->dt_ = cur_state->plan_dt;
        for (int foot = 0; foot < 4; foot++) {
            this->leg_data_[foot].q(0) = 0.0;
            this->leg_data_[foot].q(1) = 30/quad::Rad2Deg;
            this->leg_data_[foot].q(2) = -60/quad::Rad2Deg;
            this->leg_data_[foot].p = this->model_kinematic->ForwardKinematic(foot, this->leg_data_[foot].q);
            ComputeJacobian(this->leg_data_[foot].q, &this->leg_data_[foot].J_temp, foot);
            this->leg_data_[foot].J_temp_inv = this->leg_data_[foot].J_temp.inverse();
            this->leg_data_[foot].J <<  0, this->leg_data_[foot].J_temp(0,0), this->leg_data_[foot].J_temp(0,1),
                    0, 0, 0,
                    0, this->leg_data_[foot].J_temp(1,0), this->leg_data_[foot].J_temp(1, 1);
            this->leg_data_[foot].J_inv <<  0, 0, 0,
                    this->leg_data_[foot].J_temp_inv(0,0), 0, this->leg_data_[foot].J_temp_inv(0,1),
                    this->leg_data_[foot].J_temp_inv(1,0), 0, this->leg_data_[foot].J_temp_inv(1,1);

            //update velocity by Jacobian
            Vec2 temp_v;
            temp_v.x() = this->leg_data_[foot].qd.y();
            temp_v.y() = this->leg_data_[foot].qd.z();
            Vec2 temp_ = this->leg_data_[foot].J_temp * temp_v;
            this->leg_data_[foot].v << temp_.x(), 0.0, temp_.y();
            /*! Update to state interior */
            cur_state->foot_p_robot.block<3, 1>(0, foot) = this->leg_data_[foot].p;
            cur_state->foot_v_robot.block<3, 1>(0, foot) << 0, 0, 0;
            cur_state->foot_q.block<3, 1>(0, foot) = this->leg_data_[foot].q;
            cur_state->foot_qd.block<3, 1>(0, foot) = this->leg_data_[foot].qd;
            cur_state->foot_p_abs.block<3, 1>(0, foot) =
                    cur_state->rotate_matrix * cur_state->foot_p_robot.block<3, 1>(0, foot);
            cur_state->foot_p.block<3, 1>(0, foot) =
                    cur_state->foot_p_abs.block<3, 1>(0, foot) + cur_state->cur_position;
        }


    }

    void UpdateData(){
        for (int foot = 0; foot < 4; foot++) {
            this->leg_data_[foot].p = this->model_kinematic->ForwardKinematic(foot, this->leg_data_[foot].q);
            ComputeJacobian(this->leg_data_[foot].q, &this->leg_data_[foot].J_temp, foot);
            this->leg_data_[foot].J_temp_inv = this->leg_data_[foot].J_temp.inverse();
            this->leg_data_[foot].J <<  0, this->leg_data_[foot].J_temp(0,0), this->leg_data_[foot].J_temp(0,1),
                    0, 0, 0,
                    0, this->leg_data_[foot].J_temp(1,0), this->leg_data_[foot].J_temp(1, 1);
            this->leg_data_[foot].J_inv <<  0, 0, 0,
                    this->leg_data_[foot].J_temp_inv(0,0), 0, this->leg_data_[foot].J_temp_inv(0,1),
                    this->leg_data_[foot].J_temp_inv(1,0), 0, this->leg_data_[foot].J_temp_inv(1,1);
            //this->leg_data_[foot].J_inv = this->leg_data_[foot].J.inverse();
            //update velocity by Jacobian
            Vec2 temp_v;
            temp_v.x() = this->leg_data_[foot].qd.y();
            temp_v.y() = this->leg_data_[foot].qd.z();
            Vec2 temp_ = this->leg_data_[foot].J_temp * temp_v;
            this->leg_data_[foot].v << temp_.x(), 0.0, temp_.y();
        }
    }

    void UpdateCommand(){
        for (int foot = 0; foot < 4; foot++) {
            this->leg_command_[foot].qDes = this->model_kinematic->InverseKinematic(foot, this->leg_command_[foot].pDes);
            Vec2 temp_v;
            temp_v.x() = this->leg_command_[foot].vDes.x();
            temp_v.y() = this->leg_command_[foot].vDes.z();
            Vec2 temp_ = this->leg_data_[foot].J_temp.inverse()*temp_v;
            this->leg_command_[foot].qdDes.y() = temp_.x();
            this->leg_command_[foot].qdDes.z() = temp_.y();
        }
    }

    void ComputeJacobian(Vec3 &q, Mat22* J, int leg){
        double l1 = quad::HIP_LENGTH;
        double l2 = quad::KNEE_LENGTH;

        double s1 = std::sin(q(1));
        double s2 = std::sin(q(2));
        double c1 = std::cos(q(1));
        double c2 = std::cos(q(2));
        double s12 = s1*c2+c1*s2;
        double c12 = c1*c2-s1*s2;

        J->operator()(0,0) = l1*c1+l2*c12;
        J->operator()(0, 1) = l2*c12;
        J->operator()(1,0) = l1*s1+l2* s12;
        J->operator()(1,1) = l2*s12;
    }

    void SetData(std_msgs::Float64MultiArray* angle_gazebo_data_){
        for (int foot = 0; foot < 4; foot++) {
            angle_gazebo_data_->data[3*foot] = this->leg_command_[foot].qDes.x()*quad::Rad2Deg;
            angle_gazebo_data_->data[3*foot+1] = this->leg_command_[foot].qDes.y()*quad::Rad2Deg;
            angle_gazebo_data_->data[3*foot+2] = this->leg_command_[foot].qDes.z()*quad::Rad2Deg;
        }
    }

};



#endif 
