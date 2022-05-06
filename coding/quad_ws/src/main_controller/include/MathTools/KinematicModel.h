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
                this->init_p_[foot].x() = -this->X_OFFSET+0.08;
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
     * Return the leg coordinate in robot frame
     */
    Vec3 ForwardKinematic(int _leg, Vec3 _q){
        this->cur_p_[_leg].x() = this->init_p_[_leg].x() -this->l1*std::sin(_q(1))
                -this->l2*std::sin(_q(1)+_q(2));
        this->cur_p_[_leg].y() = this->init_p_[_leg].y();
        this->cur_p_[_leg].z() = this->init_p_[_leg].z() - this->l1*std::cos(_q(1))
                -this->l2*std::cos(_q(1)+_q(2));
        return this->cur_p_[_leg];
    }


    /*!
     * theta1->[-pi/4,pi/4] theta2->[-pi/2, 0]
     */
    Vec3 InverseKinematic(int _leg, Vec3 _p){
        /*! Trans to leg frame */
        this->temp_p_[_leg] = _p - this->init_p_[_leg];

        /*! Get theta2 */
        double c2 = (pow(this->temp_p_[_leg].x(), 2) + pow(this->temp_p_[_leg].z(),2) - pow(this->l1, 2) - pow(this->l2, 2))
                / (2*this->l1*this->l2);
        this->des_q_[_leg].z() = acos(c2);
        if(this->des_q_[_leg].z() > 0){
            this->des_q_[_leg].z() = -this->des_q_[_leg].z();
        }
        double s2 = sin(this->des_q_[_leg].z());

        /*! Get theta1 */
        double c1 = (-(this->l1+l2*c2)*this->temp_p_[_leg].z() - this->l2*s2*this->temp_p_[_leg].x())
                / (pow(this->temp_p_[_leg].x(), 2) + pow(this->temp_p_[_leg].z(),2));
        this->des_q_[_leg].y() = acos(c1);

        /*! Verify the theta1 */
        double x_output, z_output;
        x_output = -this->l1*sin(this->des_q_[_leg].y()) - this->l2*sin(this->des_q_[_leg].y()+this->des_q_[_leg].z());
        z_output = -this->l1*cos(this->des_q_[_leg].y()) - this->l2*cos(this->des_q_[_leg].y()+this->des_q_[_leg].z());
        if(abs(x_output-this->temp_p_[_leg].x())>0.02 || abs(z_output-this->temp_p_[_leg].z())>0.02){
            this->des_q_[_leg].y() = -this->des_q_[_leg].y();
            x_output = -this->l1*sin(this->des_q_[_leg].y()) - this->l2*sin(this->des_q_[_leg].y()+this->des_q_[_leg].z());
            z_output = -this->l1*cos(this->des_q_[_leg].y()) - this->l2*cos(this->des_q_[_leg].y()+this->des_q_[_leg].z());
        }
        /*! test */
        //std::cout << "foot" << _leg << std::endl;
        //std::cout << "theta1 = " << this->des_q_[_leg].y() << " theta2 = " << this->des_q_[_leg].z() << std::endl;
        //std::cout << "x = " << this->temp_p_[_leg].x() << " z = " << this->temp_p_[_leg].z() << std::endl;
        //std::cout << "x_output = " << x_output << " z_output = " << z_output << std::endl;

        return this->des_q_[_leg];


    }

};


#endif 
