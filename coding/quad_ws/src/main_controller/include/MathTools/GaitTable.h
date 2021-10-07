//
// Created by demphi on 2021/10/2.
//

#ifndef _GAITTABLE_
#define _GAITTABLE_
#include "reference/common_include.h"

/**
 * @brief Hopf oscillator calculated from matlab and form a time table to search
 */
class GaitTable{
public:
    int count = 0;
    bool is_init = 0;
    std_msgs::Float64MultiArray angle;

    //角度制 table
    vector<float> walk_table_thigh = {
            -1.593, -2.104, -2.592, -3.055, -3.512, -3.960, -4.394, -4.813,
            -5.212, -5.277, -5.589, -5.939, -6.263, -6.553, -6.812, -7.032,
            -7.217, -7.360, -7.401, -7.467, -7.514, -7.433, -7.115, -6.614,
            -5.927, -5.092, -4.127, -3.609, -3.071, -1.948, -0.793, 0.367 ,
            1.507 , 2.598 , 3.625 , 4.554 , 5.116 , 5.381 , 6.079 , 6.651 ,
            7.073 , 7.354 , 7.480 , 7.504 , 7.483 , 7.454 , 7.448 , 7.403 ,
            7.352 , 7.292 , 7.223 , 7.143 , 7.051 , 6.944 , 6.822 , 6.683 ,
            6.526 , 6.350 , 6.154 , 5.937 , 5.697 , 5.433 , 5.160 , 5.114 ,
            4.872 , 4.563 , 4.232 , 3.880 , 3.507 , 3.115 , 2.703 , 2.273 ,
            2.126 , 1.826 , 1.363 , 0.888 , 0.402 , -0.091, -0.590, -1.092
    };
    vector<float> walk_table_leg = {
            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , -0.808, -1.724, -2.552,
            -3.296, -3.946, -4.478, -4.701, -4.893, -5.174, -5.329, -5.349,
            -5.249, -5.024, -4.693, -4.257, -3.918, -3.738, -3.140, -2.486,
            -1.787, -1.072, -0.409, 0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,
            0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000
    };

    GaitTable();
    ~GaitTable();
    std_msgs::Float64MultiArray Get_Angle_From_Table();
};

GaitTable::GaitTable() {
    this->is_init = true;
    this->angle.data.resize(12);
    this->angle.data[0] = quad::DEFAULT_SHOULD_ANGLE;
    this->angle.data[3] = quad::DEFAULT_SHOULD_ANGLE;
    this->angle.data[6] = quad::DEFAULT_SHOULD_ANGLE;
    this->angle.data[9] = quad::DEFAULT_SHOULD_ANGLE;
}

GaitTable::~GaitTable() {

}

std_msgs::Float64MultiArray GaitTable::Get_Angle_From_Table() {
    if(!this->is_init){
        ROS_INFO("Init failed!");
    }
    // table 0->79
    if (this->count == 80){
        count = 0;
    }
    //LF
    this->angle.data[1] = this->walk_table_thigh[count];
    this->angle.data[2] = this->walk_table_leg[count];

    //RF
    this->angle.data[4] = this->walk_table_thigh[(count+40)%80];
    this->angle.data[5] = this->walk_table_leg[(count+40)%80];

    //RH
    this->angle.data[7] = this->walk_table_thigh[(count+60)%80];
    this->angle.data[8] = this->walk_table_leg[(count+60)%80];

    //LH
    this->angle.data[10] = this->walk_table_thigh[(count+20)%80];
    this->angle.data[11] = this->walk_table_leg[(count+20)%80];

    this->count++;

    return this->angle;
}




#endif
