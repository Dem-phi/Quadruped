//
// Created by demphi on 2021/10/2.
//

#ifndef _HOPF_
#define _HOPF_
#include "reference/common_include.h"
#include "math.h"

using namespace Eigen;
/**
 * @brief Hopf transplant from matlab
 */
class Hopf{
public:

    // rad to deg -> deg = rad * Rad2Deg
    // deg to rad -> rad = deg / Rad2Deg
    const double Rad2Deg = 45.0/ atan(1);

    // define some const
    const float omiga_sw = 5 * M_PI;
    const float alpha = 10000;
    const float a = 100;
    const float theta0 = M_PI/6;

    // LF 1, RF 2, RH 3, LH 4
    Eigen::Vector4f phi = {0.0, 0.0, 0.0, 0.0};
    float omiga_st = 0;
    float mu = 0;
    float beta = 0;
    Mat44f M_cos;
    Mat44f M_sin;
    Eigen::Vector4f RX = {0.0, 0.0, 0.0, 0.0};
    Eigen::Vector4f RY = {0.0, 0.0, 0.0, 0.0};

    //1000HZ
    float delta_t = 0.002;

    //output
    Eigen::Vector4f x = {0.1310, 0, 0, 0};
    Eigen::Vector4f y = {0, 0, 0, 0};
    std_msgs::Float64MultiArray angle_msg;

    //amplitude[1] = A_h; amplitude[2] = A_k;
    Eigen::Vector2f amplitude = {0.0, 0.0};

    Hopf(Eigen::Vector2f amplitude, float beta, Eigen::Vector4f phi);
    ~Hopf();
    void UpdateRotateMatrix();
    std_msgs::Float64MultiArray CalculateAngle(std_msgs::Float64MultiArray msg);

};

Hopf::Hopf(Eigen::Vector2f amplitude, float beta, Eigen::Vector4f phi){
    this->amplitude.x() = amplitude.x()/this->Rad2Deg;
    this->amplitude.y() = amplitude.y()/this->Rad2Deg;
    this->mu = pow(this->amplitude.x(), 2);
    this->beta = beta;
    this->phi = phi;
    this->omiga_st = ((1- this->beta)/this->beta)*this->omiga_sw;

    //init the R(theta) Matrix
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            this->M_cos(i, j)= cos(2*M_PI*(this->phi[j]-this->phi[i]));
            this->M_sin(i, j)= sin(2*M_PI*(this->phi[j]-this->phi[i]));
        }
        // init the first value of output
    }

    this->angle_msg.data.resize(12);
    this->angle_msg.data[0] = quad::DEFAULT_SHOULD_ANGLE;
    this->angle_msg.data[3] = quad::DEFAULT_SHOULD_ANGLE;
    this->angle_msg.data[6] = quad::DEFAULT_SHOULD_ANGLE;
    this->angle_msg.data[9] = quad::DEFAULT_SHOULD_ANGLE;
}

Hopf::~Hopf() {

}

/**
 * update the value of R(\theta)*[xj, yj]'
 */
void Hopf::UpdateRotateMatrix() {
    Eigen::Vector4f temp_x, temp_y;
    temp_x = {0.0, 0.0, 0.0, 0.0};
    temp_y = {0.0, 0.0, 0.0, 0.0};

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            temp_x[i] += this->M_cos(j, i)*this->x[j] - this->M_sin(j, i)*this->y[j];
            temp_y[i] += this->M_sin(j, i)*this->x[j] + this->M_cos(j, i)*this->y[j];
        }
    }
    for(int i = 0; i < 4; i++){
        this->RX[i] = temp_x[i];
        this->RY[i] = temp_y[i];
    }

}

std_msgs::Float64MultiArray Hopf::CalculateAngle(std_msgs::Float64MultiArray msg) {
    UpdateRotateMatrix();
    for(int i = 0; i < 4; i++){
        float omiga = this->omiga_st/(exp(-1*this->a*this->y[i])+1)
                    + this->omiga_sw/(exp(this->a*this->y[i])+1);
        float r = pow(this->x[i], 2) + pow(this->y[i], 2);
        float delta_x = this->alpha*(this->mu-r)*this->x[i]
                        - omiga*this->y[i] + this->RX[i];
        float delta_y = this->alpha*(this->mu-r)*this->y[i]
                        + omiga*this->x[i] + this->RY[i];
        this->x[i] += delta_x * this->delta_t;
        this->y[i] += delta_y * this->delta_t;
    }
    //rad
    this->angle_msg.data[1] = this->x[0]*this->Rad2Deg;
    this->angle_msg.data[2] = -this->y[0]*this->amplitude[1]/this->amplitude[0]*this->Rad2Deg;
    this->angle_msg.data[4] = this->x[1]*this->Rad2Deg;
    this->angle_msg.data[5] = -this->y[1]*this->amplitude[1]/this->amplitude[0]*this->Rad2Deg;
    this->angle_msg.data[7] = this->x[2]*this->Rad2Deg;
    this->angle_msg.data[8] = -this->y[2]*this->amplitude[1]/this->amplitude[0]*this->Rad2Deg;
    this->angle_msg.data[10] = this->x[3]*this->Rad2Deg;
    this->angle_msg.data[11] = -this->y[3]*this->amplitude[1]/this->amplitude[0]*this->Rad2Deg;

    for(int i=0; i<4; i++){
        if(this->y[i] > 0){
           this->angle_msg.data[3*i+2] = 0;
        }
    }
    return this->angle_msg;
}


#endif
