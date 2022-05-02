//
// Created by demphi on 2022/3/23.
//

#ifndef _FORWARDKINEMATIC_
#define _FORWARDKINEMATIC_

#include "common_include.h"

class KinematicModel{
public:
    const double X_OFFSET = quad::X_OFFSET;
    const double Y_OFFSET = quad::Y_OFFSET;
    double l1 = quad::HIP_LENGTH;
    double l2 = quad::KNEE_LENGTH;
    Vec3 init_p_[4];
    Vec3 cur_p_[4];
    Vec3 temp_p_[4];
    Vec3 des_q_[4];

    KinematicModel(){
        Initialize();
    }
    ~KinematicModel(){

    }

    void Initialize(){
        for (int foot = 0; foot < 4; foot++) {
            this->init_p_[foot] = Vec3::Zero();
            this->cur_p_[foot] = Vec3::Zero();
            this->des_q_[foot] = Vec3::Zero();
            // Set X axis coordinate
            if(foot == 0 || foot == 1){
                this->init_p_[foot].x() = this->X_OFFSET;
            }else{
                this->init_p_[foot].x() = -this->X_OFFSET;
            }
            // Set Y axis coordinate
            if(foot == 0 || foot == 3){
                this->init_p_[foot].y() = this->Y_OFFSET;
            }else{
                this->init_p_[foot].y() = -this->Y_OFFSET;
            }

        }

    }

    /*!
     * Return the leg coordinate in body frame
     */
    Vec3 ForwardKinematic(int _leg, Vec3 _q){
        this->cur_p_[_leg].x() = this->init_p_[_leg].x() - this->l1*std::sin(_q(1))
                -this->l2*std::sin(_q(1)+_q(2));
        this->cur_p_[_leg].y() = this->init_p_[_leg].y();
        this->cur_p_[_leg].z() = this->init_p_[_leg].z() - this->l1*std::cos(_q(1))
                -this->l2*std::cos(_q(1)+_q(2));
        return this->cur_p_[_leg];
    }


    /*!
     * theta1->[0,pi/4] theta2->[-pi/2, 0]
     */
    Vec3 InverseKinematic(int _leg, Vec3 _p){
        if(_leg == 0 || _leg == 1){
            this->temp_p_[_leg].x() = _p.x()-this->init_p_[_leg].x();
        }else{
            /*!暂时用0 1 腿的数据处理后两个腿的运动学模型*/
            if(_leg == 2){
                this->temp_p_[_leg].x() = this->temp_p_[0].x();
            }
            if(_leg == 3){
                this->temp_p_[_leg].x() = this->temp_p_[1].x();
            }
            //this->temp_p_[_leg].x() = _p.x()+this->init_p_[_leg].x();
        }
        this->temp_p_[_leg].z() = _p.z();
        double c2 = (pow(this->temp_p_[_leg].x(), 2) + pow(this->temp_p_[_leg].z(),2) - pow(this->l1, 2) - pow(this->l2, 2))
                / (2*this->l1*this->l2);
        this->des_q_[_leg].z() = acos(c2);
        if(this->des_q_[_leg].z() > 0){
            this->des_q_[_leg].z() = -this->des_q_[_leg].z();
        }
        double s2 = sin(this->des_q_[_leg].z());

        this->des_q_[_leg].y() = atan2(this->temp_p_[_leg].x(), this->temp_p_[_leg].z())
                - atan2(this->l2*s2, this->l1+this->l2*c2);
        if(this->des_q_[_leg].y() < 0){
            this->des_q_[_leg].y() = M_PI*2+this->des_q_[_leg].y();
        }
        if(this->des_q_[_leg].y() > M_PI){
            this->des_q_[_leg].y() = this->des_q_[_leg].y()-M_PI;
        }

        return this->des_q_[_leg];
    }

};


#endif 
