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
        J = Mat22::Zero();
    }
    Vec3 p, v;
    Mat22 J;
    /*! angle of three joints*/
    Vec3 q;
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
    double step_size_, period_time_, velocity_;
    double dt_;
    LegControllerCommand leg_command_[4];
    LegControllerData leg_data_[4];

    KinematicModel* model_kinematic = new KinematicModel();

    LegController(double _velocity, double _period_time, double _dt){
        for (int i = 0; i < 4; i++) {
            this->leg_command_[i].zero();
            this->leg_data_[i].zero();
        }
        this->velocity_ = _velocity;
        this->period_time_ = _period_time;
        this->dt_ = _dt;
        this->step_size_ = this->velocity_*this->period_time_;
    }
    ~LegController(){
        delete this->model_kinematic;
    }

    void FirstUpdateData(){
        for (int foot = 0; foot < 4; foot++) {
            this->leg_data_[foot].q(0) = 0.0;
            if(foot == 0 || foot == 2){
                this->leg_data_[foot].q(1) = 18.12/quad::Rad2Deg;
            }else{
                this->leg_data_[foot].q(1) = 38.6/quad::Rad2Deg;
            }
            this->leg_data_[foot].q(2) = -56.7/quad::Rad2Deg;
            this->leg_data_[foot].p = this->model_kinematic->ForwardKinematic(foot, this->leg_data_[foot].q);
            ComputeJacobian(this->leg_data_[foot].q, &this->leg_data_[foot].J, foot);

            //update velocity by Jacobian
            this->leg_data_[foot].v << 0.0, 0.0, 0.0;
        }
    }


    void UpdateData(){
        for (int foot = 0; foot < 4; foot++) {
            for(int joint = 0; joint < 3; joint++){
                // Assume the command is fully completed
                this->leg_data_[foot].q(joint) = this->leg_command_[foot].qDes(joint);
            }
            this->leg_data_[foot].p = this->model_kinematic->ForwardKinematic(foot, this->leg_data_[foot].q);
            ComputeJacobian(this->leg_data_[foot].q, &this->leg_data_[foot].J, foot);

            //update velocity by Jacobian
            this->leg_data_[foot].v << 0.0, 0.0, 0.0;
        }
    }

    void UpdateCommand(){
/*        for (int foot = 0; foot < 4; foot++) {
            this->leg_command_[foot].qDes = this->leg_data_[foot].q;
            Vec2 temp_v;
            temp_v.x() = this->leg_command_[foot].vDes.x();
            temp_v.y() = this->leg_command_[foot].vDes.z();
            Vec2 temp_ = this->leg_data_[foot].J.inverse()*temp_v*this->dt_;
            this->leg_command_[foot].qDes.y() += temp_.x();
            this->leg_command_[foot].qDes.z() += temp_.y();

            // add a PD controller

        }*/
        for (int foot = 0; foot < 4; foot++) {
            this->leg_command_[foot].qDes = this->model_kinematic->InverseKinematic(foot, this->leg_command_[foot].pDes);
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
