//
// Created by demphi on 2021/10/3.
//

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"


// LF -> RF -> RH -> RF
// shoulder -> thigh -> knee
std_msgs::Float64 real_angle[12];
ros::Publisher remap[12];
ros::Subscriber angle_sub;

//rad to deg -> deg = rad * Rad2Deg
//deg to rad -> rad = deg / Rad2Deg
double Rad2Deg = 45.0/ atan(1.0);

void angle_Callback(const std_msgs::Float64MultiArrayConstPtr &msg){
    real_angle[0].data = msg->data[0]/Rad2Deg;
    real_angle[1].data = msg->data[1]/Rad2Deg;
    real_angle[2].data = msg->data[2]/Rad2Deg;
    real_angle[3].data = msg->data[3]/Rad2Deg;
    real_angle[4].data = msg->data[4]/Rad2Deg;
    real_angle[5].data = msg->data[5]/Rad2Deg;
    real_angle[6].data = msg->data[6]/Rad2Deg;
    real_angle[7].data = msg->data[7]/Rad2Deg;
    real_angle[8].data = msg->data[8]/Rad2Deg;
    real_angle[9].data = msg->data[9]/Rad2Deg;
    real_angle[10].data = msg->data[10]/Rad2Deg;
    real_angle[11].data = msg->data[11]/Rad2Deg;

    for(int i = 0;i < 12; i++){
        remap[i].publish(real_angle[i]);
    }
    return;
}

int main (int argc, char** argv){
    ros::init(argc, argv, "remap");
    ros::NodeHandle nh;
    angle_sub = nh.subscribe("/quad/set_angle_gazebo", 1, angle_Callback);
/*    remap[0] = nh.advertise<std_msgs::Float64>("/cheetah/LF_shoulder_position_controller/command", 1);
    remap[1] = nh.advertise<std_msgs::Float64>("/cheetah/LF_hip_position_controller/command", 1);
    remap[2] = nh.advertise<std_msgs::Float64>("/cheetah/LF_knee_position_controller/command", 1);
    remap[3] = nh.advertise<std_msgs::Float64>("/cheetah/RF_shoulder_position_controller/command", 1);
    remap[4] = nh.advertise<std_msgs::Float64>("/cheetah/RF_hip_position_controller/command", 1);
    remap[5] = nh.advertise<std_msgs::Float64>("/cheetah/RF_knee_position_controller/command", 1);
    remap[6] = nh.advertise<std_msgs::Float64>("/cheetah/RH_shoulder_position_controller/command", 1);
    remap[7] = nh.advertise<std_msgs::Float64>("/cheetah/RH_hip_position_controller/command", 1);
    remap[8] = nh.advertise<std_msgs::Float64>("/cheetah/RH_knee_position_controller/command", 1);
    remap[9] = nh.advertise<std_msgs::Float64>("/cheetah/LH_shoulder_position_controller/command", 1);
    remap[10] = nh.advertise<std_msgs::Float64>("/cheetah/LH_hip_position_controller/command", 1);
    remap[11] = nh.advertise<std_msgs::Float64>("/cheetah/LH_knee_position_controller/command", 1);*/
/*    remap[3] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FR_hip_controller/command", 1);
    remap[4] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FR_thigh_controller/command", 1);
    remap[5] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FR_calf_controller/command", 1);
    remap[0] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FL_hip_controller/command", 1);
    remap[1] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FL_thigh_controller/command", 1);
    remap[2] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/FL_calf_controller/command", 1);
    remap[6] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RR_hip_controller/command", 1);
    remap[7] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RR_thigh_controller/command", 1);
    remap[8] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RR_calf_controller/command", 1);
    remap[9] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RL_hip_controller/command", 1);
    remap[10] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RL_thigh_controller/command", 1);
    remap[11] = nh.advertise<unitree_legged_msgs::MotorCmd>("/aliengo_gazebo/RL_calf_controller/command", 1);*/
    remap[3] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/FR_hip_controller/command", 1);
    remap[4] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/FR_thigh_controller/command", 1);
    remap[5] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/FR_calf_controller/command", 1);
    remap[0] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/FL_hip_controller/command", 1);
    remap[1] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/FL_thigh_controller/command", 1);
    remap[2] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/FL_calf_controller/command", 1);
    remap[6] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/RR_hip_controller/command", 1);
    remap[7] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/RR_thigh_controller/command", 1);
    remap[8] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/RR_calf_controller/command", 1);
    remap[9] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/RL_hip_controller/command", 1);
    remap[10] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/RL_thigh_controller/command", 1);
    remap[11] = nh.advertise<std_msgs::Float64>("/aliengo_gazebo/RL_calf_controller/command", 1);

    ros::spin();
}