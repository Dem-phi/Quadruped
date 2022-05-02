//
// Created by demphi on 2022/5/1.
//

#ifndef _MAPPERTOGAZEBO_H
#define _MAPPERTOGAZEBO_H

#include "common_defination.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/MotorCmd.h"

using namespace quad;

class MapperToGazebo{
public:
    MapperToGazebo(ros::NodeHandle &_nh){
        this->nh = _nh;

        /*! Publisher */
        pub_joint_cmd[0] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FL_hip_controller/command", 1);
        pub_joint_cmd[1] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FL_thigh_controller/command", 1);
        pub_joint_cmd[2] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FL_calf_controller/command", 1);

        pub_joint_cmd[3] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FR_hip_controller/command", 1);
        pub_joint_cmd[4] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FR_thigh_controller/command", 1);
        pub_joint_cmd[5] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FR_calf_controller/command", 1);

        pub_joint_cmd[6] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RL_hip_controller/command", 1);
        pub_joint_cmd[7] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RL_thigh_controller/command", 1);
        pub_joint_cmd[8] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RL_calf_controller/command", 1);

        pub_joint_cmd[9] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RR_hip_controller/command", 1);
        pub_joint_cmd[10] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RR_thigh_controller/command", 1);
        pub_joint_cmd[11] = this->nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RR_calf_controller/command", 1);

    }

    ~MapperToGazebo(){}

    bool SendCommand(STATE_INTERIOR * state_interior){
        // send control cmd to robot via ros topic
        unitree_legged_msgs::LowCmd low_cmd;
        for (int i = 0; i < 12; i++) {
            low_cmd.motorCmd[i].mode = 0x0A;
            low_cmd.motorCmd[i].q = 0;
            low_cmd.motorCmd[i].dq = 0;
            low_cmd.motorCmd[i].Kp = 0;
            low_cmd.motorCmd[i].Kd = 0;
            low_cmd.motorCmd[i].tau = state_interior->joint_torques(i, 0);
            pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
        }
        return true;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher pub_joint_cmd[12];
};


#endif 
