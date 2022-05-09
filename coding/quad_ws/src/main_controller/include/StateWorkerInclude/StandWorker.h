//
// Created by demphi on 2021/10/1.
//

#ifndef _STANDWORKER_
#define _STANDWORKER_

#include "StateWorker.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64MultiArray.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/MotorCmd.h"

/**
 * @brief send angle msg and stand
 * @param none
 */
class StandWorker: public StateWorker{
private:
    /*! max = pi/6*/
    int flag_ = 0;
    int count_;
    double command_hip_ = 0;
    double command_knee_ = 0;
    double init_time_, cur_time_, end_time_;
    double init_hip_ = 90.0, init_knee_ = -175.0;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
    Eigen::Matrix<float, 1, NUM_LEG> spline_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;

public:
    ros::NodeHandle nh_;
    ros::Publisher pub_joint_cmd[12];
    BezierUtils bezierUtils[NUM_LEG];

    virtual void run(STATE_INTERIOR *cur_state);
    virtual bool is_finished();

    StandWorker(ros::NodeHandle &nh);
    ~StandWorker();

};

StandWorker::StandWorker(ros::NodeHandle &nh) {
    this->nh_ = nh;
    this->pub_joint_cmd[0] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_hip_controller/command", 1);
    this->pub_joint_cmd[1] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_thigh_controller/command", 1);
    this->pub_joint_cmd[2] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_calf_controller/command", 1);
    this->pub_joint_cmd[3] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_hip_controller/command", 1);
    this->pub_joint_cmd[4] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_thigh_controller/command", 1);
    this->pub_joint_cmd[5] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_calf_controller/command", 1);
    this->pub_joint_cmd[6] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    this->pub_joint_cmd[7] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    this->pub_joint_cmd[8] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);
    this->pub_joint_cmd[9] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    this->pub_joint_cmd[10] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    this->pub_joint_cmd[11] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);

    /*! Init some param*/
    this->init_time_ = ros::Time::now().toSec();
    this->end_time_ = 0.0;

    this->foot_pos_cur.setZero();
    this->foot_vel_cur.setZero();
    this->spline_time.setZero();
    this->foot_pos_target.setZero();
    this->foot_vel_target.setZero();
    this->foot_pos_error.setZero();
    this->foot_vel_error.setZero();
}

StandWorker::~StandWorker() {

}

void StandWorker::run(STATE_INTERIOR *cur_state) {
    if(this->count_++ > 100){
        ROS_INFO("Standing");
        this->count_ = 0;
    }
    for (int foot = 0; foot < NUM_LEG; foot++) {
        cur_state->contacts[foot] = 1;
    }
    cur_state->gait_counter_reset();

/*

    this->cur_time_ = ros::Time::now().toSec();
    if (this->cur_time_-this->init_time_>=5.0 && this->flag_ == 0){
        this->flag_ = 1;
    }
    // Stand in gazebo

    if(this->flag_ == 1){
        for (int i = 0; i < 4; i++) {
            this->low_cmd.motorCmd[3*i].mode = 0x0A;
            this->low_cmd.motorCmd[3*i].q = 0;
            this->low_cmd.motorCmd[3*i].dq = 0;
            this->low_cmd.motorCmd[3*i].Kp = 70;
            this->low_cmd.motorCmd[3*i].Kd = 3;
            this->low_cmd.motorCmd[3*i].tau = 0;

            this->low_cmd.motorCmd[3*i+1].mode = 0x0A;
            this->low_cmd.motorCmd[3*i+1].q = (this->init_hip_-this->command_hip_)/Rad2Deg;
            this->low_cmd.motorCmd[3*i+1].dq = 0;
            this->low_cmd.motorCmd[3*i+1].Kp = 180;
            this->low_cmd.motorCmd[3*i+1].Kd = 8;
            this->low_cmd.motorCmd[3*i+1].tau = 0;

            this->low_cmd.motorCmd[3*i+2].mode = 0x0A;
            this->low_cmd.motorCmd[3*i+2].q = (this->init_knee_+2*this->command_knee_)/Rad2Deg;
            this->low_cmd.motorCmd[3*i+2].dq = 0;
            this->low_cmd.motorCmd[3*i+2].Kp = 300;
            this->low_cmd.motorCmd[3*i+2].Kd = 15;
            this->low_cmd.motorCmd[3*i+2].tau = 0;

        }
        if(this->command_hip_ < 60.0 ){
            this->command_hip_ = this->command_hip_+0.3;
        }
        if(this->command_knee_ < 57.5){
            this->command_knee_ = this->command_knee_ + 0.3;
        }
        if(this->command_hip_ >=60.0 && this->command_knee_>=57.5){
            this->flag_ = 2;
            this->end_time_ = ros::Time::now().toSec();
        }
    }
    else if(this->flag_ == 0){
        for (int i = 0; i < 4; i++) {
            this->low_cmd.motorCmd[3*i].mode = 0x0A;
            this->low_cmd.motorCmd[3*i].q = 0;
            this->low_cmd.motorCmd[3*i].dq = 0;
            this->low_cmd.motorCmd[3*i].Kp = 70;
            this->low_cmd.motorCmd[3*i].Kd = 3;
            this->low_cmd.motorCmd[3*i].tau = 0;

            this->low_cmd.motorCmd[3*i+1].mode = 0x0A;
            this->low_cmd.motorCmd[3*i+1].q = this->init_hip_/Rad2Deg;
            this->low_cmd.motorCmd[3*i+1].dq = 0;
            this->low_cmd.motorCmd[3*i+1].Kp = 180;
            this->low_cmd.motorCmd[3*i+1].Kd = 8;
            this->low_cmd.motorCmd[3*i+1].tau = 0;

            this->low_cmd.motorCmd[3*i+2].mode = 0x0A;
            this->low_cmd.motorCmd[3*i+2].q = this->init_knee_/Rad2Deg;
            this->low_cmd.motorCmd[3*i+2].dq = 0;
            this->low_cmd.motorCmd[3*i+2].Kp = 300;
            this->low_cmd.motorCmd[3*i+2].Kd = 15;
            this->low_cmd.motorCmd[3*i+2].tau = 0;
        }
    }
    for (int i = 0; i < 12; i++) {
        this->pub_joint_cmd[i].publish(this->low_cmd.motorCmd[i]);
    }
*/

    cur_state->foot_pDes_robot = cur_state->foot_p_bias;
    for (int foot = 0; foot < NUM_LEG; foot++){
        double delta_x = sqrt(abs(cur_state->foot_p_bias.block<3, 1>(0, foot).z())/9.8)*(cur_state->cur_vel_b.x()-cur_state->command_vel.x())
                         +0.2/2*cur_state->command_vel.x();
        double delta_y = sqrt(abs(cur_state->foot_p_bias.block<3, 1>(0, foot).z())/9.8)*(cur_state->cur_vel_b.y()-cur_state->command_vel.y())
                         +0.2/2*cur_state->command_vel.y();
        if (delta_x < -FOOT_DELTA_X_LIMIT) {
            delta_x = -FOOT_DELTA_X_LIMIT;
        }
        if (delta_x > FOOT_DELTA_X_LIMIT) {
            delta_x = FOOT_DELTA_X_LIMIT;
        }
        if (delta_y < -FOOT_DELTA_Y_LIMIT) {
            delta_y = -FOOT_DELTA_Y_LIMIT;
        }
        if (delta_y > FOOT_DELTA_Y_LIMIT) {
            delta_y = FOOT_DELTA_Y_LIMIT;
        }
        cur_state->foot_pDes_robot(0, foot) += delta_x;
        cur_state->foot_pDes_robot(1, foot) += delta_y;

        cur_state->foot_pDes_abs.block<3, 1>(0, foot) = cur_state->rotate_matrix * cur_state->foot_pDes_robot.block<3, 1>(0, foot);
        cur_state->foot_pDes.block<3, 1>(0, foot) = cur_state->foot_pDes_abs.block<3, 1>(0, foot) + cur_state->cur_position;
    }


    for (int foot = 0; foot < NUM_LEG; foot++) {
        this->foot_pos_cur.block<3, 1>(0, foot) = cur_state->rotate_matrix_z.transpose() *cur_state->foot_p_abs.block<3, 1>(0, foot);
        spline_time(foot) = 0.0;
        cur_state->foot_pos_start.block<3, 1>(0, foot) = foot_pos_cur.block<3, 1>(0, foot);

        foot_pos_target.block<3, 1>(0, foot) = bezierUtils[foot].get_foot_pos_curve(
                spline_time(foot),
                cur_state->foot_pos_start.block<3, 1>(0, foot),
                cur_state->foot_pDes_robot.block<3, 1>(0, foot),
                0.0);

        foot_vel_cur.block<3, 1>(0, foot) =
                (foot_pos_cur.block<3, 1>(0, foot) - cur_state->foot_pos_rel_last_time.block<3, 1>(0, foot)) / cur_state->plan_dt;
        cur_state->foot_pos_rel_last_time.block<3, 1>(0, foot) = foot_pos_cur.block<3, 1>(0, foot);

        foot_vel_target.block<3, 1>(0, foot) =
                (foot_pos_target.block<3, 1>(0, foot) - cur_state->foot_pos_target_last_time.block<3, 1>(0, foot)) / cur_state->plan_dt;
        cur_state->foot_pos_target_last_time.block<3, 1>(0, foot) = foot_pos_target.block<3, 1>(0, foot);

        foot_pos_error.block<3, 1>(0, foot) = foot_pos_target.block<3, 1>(0, foot) - foot_pos_cur.block<3, 1>(0, foot);
        foot_vel_error.block<3, 1>(0, foot) = foot_vel_target.block<3, 1>(0, foot) - foot_vel_cur.block<3, 1>(0, foot);
        cur_state->foot_forces_swing.block<3, 1>(0, foot) =
                foot_pos_error.block<3, 1>(0, foot).cwiseProduct(cur_state->kp_foot.block<3, 1>(0, foot))
                + foot_vel_error.block<3, 1>(0, foot).cwiseProduct(cur_state->kd_foot.block<3, 1>(0, foot));
    }

    /*! Change Gait Type */
    if(cur_state->gait_type != quad::STAND){
        this->flag_ = 2;
    }
}

bool StandWorker::is_finished() {
    if(this->flag_ == 2){
        ROS_INFO("Finish Stand State");
        return true;
    }else{
        return false;
    }

}

#endif
