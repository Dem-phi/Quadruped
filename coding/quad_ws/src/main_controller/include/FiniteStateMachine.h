//
// Created by demphi on 2021/10/1.
//

#ifndef _FINITESTATEMACHINE_
#define _FINITESTATEMACHINE_

#include "stdarg.h"
#include "Workers_common_include.h"
#include "unitree_legged_msgs/MotorState.h"

class FSM{
private:
    /*! main loop function runs in a specific frequency */
    ros::Timer FSM_Timer_, FSM_Timer2_;
    /*! Publish Interior msg */
    ros::Publisher position_pub_, velocity_pub_, euler_pub_;
    /*! Publish Control msg */
    ros::Publisher angle_gazebo_pub_, angle_real_pub_;
    ros::Publisher pub_joint_cmd[12];

    ros::Subscriber motor_sub_, imu_sub_, joy_sub_;
    ros::Subscriber sub_joint_msg[12], sub_foot_contact_msg[4];

    //filters
    MovingWindowFilter acc_x;
    MovingWindowFilter acc_y;
    MovingWindowFilter acc_z;
    MovingWindowFilter gyro_x;
    MovingWindowFilter gyro_y;
    MovingWindowFilter gyro_z;
    MovingWindowFilter quat_w;
    MovingWindowFilter quat_x;
    MovingWindowFilter quat_y;
    MovingWindowFilter quat_z;


public:
    ros::NodeHandle nh_;
    std::vector<StateWorker*> Workers;
    int flow = 0;

    quad::STATE_INFO state_info_;

    STATE_INTERIOR state_interior_ ;


    /*! Class for control */
    A1BasicEKF* model_StateEstimate = new A1BasicEKF();
    LegController* model_LegController = new LegController();
    Solver mpc_solver;

    FSM(ros::NodeHandle &nh);
    ~FSM();
    void loop(double dt);
    void Update_MPC(double dt);

    void set_timer();
    void set_timer2();
    void build_ScheduleTable(int Schedule, ...);

    /*! Update Data to some class*/
    void Update_LegController();
    void Update_StateEstimate();
    void SendCommand();

    /*! Callback Function */
    void MotorCallback(const std_msgs::Float64MultiArray &msg);
    void IMUCallback(const sensor_msgs::Imu &msg);
    void JoyCallback(const sensor_msgs::Joy &msg);

    /*! Callback of joint messages */
    void FL_hip_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void FL_thigh_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void FL_calf_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void FR_hip_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void FR_thigh_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void FR_calf_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void RL_hip_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void RL_thigh_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void RL_calf_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void RR_hip_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void RR_thigh_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void RR_calf_state_callback(const unitree_legged_msgs::MotorState &joint_state);
    void FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);
    void FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);
    void RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force);
    void RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force);
};

FSM::FSM(ros::NodeHandle &nh) {
    this->nh_ = nh;
    this->position_pub_ = this->nh_.advertise<geometry_msgs::Pose>(
            "/quad/Position", 1);
    this->velocity_pub_ = this->nh_.advertise<geometry_msgs::Vector3>(
            "/quad/Velocity", 1);
    this->euler_pub_ = this->nh_.advertise<geometry_msgs::Vector3>(
            "/quad/EulerAngle", 1);

    this->angle_gazebo_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle_gazebo", 1);
    this->angle_real_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle", 1);

    this->pub_joint_cmd[0] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_hip_controller/command", 1);
    this->pub_joint_cmd[1] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_thigh_controller/command", 1);
    this->pub_joint_cmd[2] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FL_calf_controller/command", 1);
    this->pub_joint_cmd[3] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_hip_controller/command", 1);
    this->pub_joint_cmd[4] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_thigh_controller/command", 1);
    this->pub_joint_cmd[5] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/FR_calf_controller/command", 1);
    this->pub_joint_cmd[6] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    this->pub_joint_cmd[7] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    this->pub_joint_cmd[8] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);
    this->pub_joint_cmd[9] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    this->pub_joint_cmd[10] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    this->pub_joint_cmd[11] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);


    this->motor_sub_ = this->nh_.subscribe("/quad/motor_info", 2, &FSM::MotorCallback ,this);
    this->imu_sub_ = this->nh_.subscribe("/body_imu", 2, &FSM::IMUCallback, this);
    this->joy_sub_ = this->nh_.subscribe("/joy", 2, &FSM::JoyCallback, this);

    this->sub_joint_msg[0] = this->nh_.subscribe("/a1_gazebo/FL_hip_controller/state", 2, &FSM::FL_hip_state_callback, this);
    this->sub_joint_msg[1] = this->nh_.subscribe("/a1_gazebo/FL_thigh_controller/state", 2, &FSM::FL_thigh_state_callback, this);
    this->sub_joint_msg[2] = this->nh_.subscribe("/a1_gazebo/FL_calf_controller/state", 2, &FSM::FL_calf_state_callback, this);
    this->sub_joint_msg[3] = this->nh_.subscribe("/a1_gazebo/FR_hip_controller/state", 2, &FSM::FR_hip_state_callback, this);
    this->sub_joint_msg[4] = this->nh_.subscribe("/a1_gazebo/FR_thigh_controller/state", 2, &FSM::FR_thigh_state_callback, this);
    this->sub_joint_msg[5] = this->nh_.subscribe("/a1_gazebo/FR_calf_controller/state", 2, &FSM::FR_calf_state_callback, this);
    this->sub_joint_msg[6] = this->nh_.subscribe("/a1_gazebo/RL_hip_controller/state", 2, &FSM::RL_hip_state_callback, this);
    this->sub_joint_msg[7] = this->nh_.subscribe("/a1_gazebo/RL_thigh_controller/state", 2, &FSM::RL_thigh_state_callback, this);
    this->sub_joint_msg[8] = this->nh_.subscribe("/a1_gazebo/RL_calf_controller/state", 2, &FSM::RL_calf_state_callback, this);
    this->sub_joint_msg[9] = this->nh_.subscribe("/a1_gazebo/RR_hip_controller/state", 2, &FSM::RR_hip_state_callback, this);
    this->sub_joint_msg[10] = this->nh_.subscribe("/a1_gazebo/RR_thigh_controller/state", 2, &FSM::RR_thigh_state_callback, this);
    this->sub_joint_msg[11] = this->nh_.subscribe("/a1_gazebo/RR_calf_controller/state", 2, &FSM::RR_calf_state_callback, this);


    this->sub_foot_contact_msg[0] = this->nh_.subscribe("/visual/FL_foot_contact/the_force", 2, &FSM::FL_foot_contact_callback, this);
    this->sub_foot_contact_msg[1] = this->nh_.subscribe("/visual/FR_foot_contact/the_force", 2, &FSM::FR_foot_contact_callback, this);
    this->sub_foot_contact_msg[2] = this->nh_.subscribe("/visual/RL_foot_contact/the_force", 2, &FSM::RL_foot_contact_callback, this);
    this->sub_foot_contact_msg[3] = this->nh_.subscribe("/visual/RR_foot_contact/the_force", 2, &FSM::RR_foot_contact_callback, this);


    this->state_interior_.Reset();
    this->state_interior_.InitParams();

    /*! Init some param */
    this->state_info_.angle_gazebo_data.data.resize(12);
    this->state_info_.angle_real_data.data.resize(12);
    this->state_info_.position_feedback_info.data.resize(12);
    this->state_info_.velocity_feedback_info.data.resize(12);
    this->state_info_.current_feedback_info.data.resize(12);
    this->acc_x = MovingWindowFilter(5);
    this->acc_y = MovingWindowFilter(5);
    this->acc_z = MovingWindowFilter(5);
    this->gyro_x = MovingWindowFilter(5);
    this->gyro_y = MovingWindowFilter(5);
    this->gyro_z = MovingWindowFilter(5);
    this->quat_w = MovingWindowFilter(5);
    this->quat_x = MovingWindowFilter(5);
    this->quat_y = MovingWindowFilter(5);
    this->quat_z = MovingWindowFilter(5);
}

FSM::~FSM() {
    // release
    for(auto each:this->Workers){
        delete each;
    }
    delete this->model_StateEstimate;
    delete this->model_LegController;
}

void FSM::loop(double dt) {
    this->state_interior_.plan_dt = dt;
    /*! running the schedule table */
    if(this->Workers[this->flow]->is_finished()){
        this->flow++;
        if(this->flow == this->Workers.size()){
            ROS_INFO("Finish ScheduleTable");
            exit(0);
        }
    }
    else{
        /*! Update Leg Control Data */
        Update_LegController();

        /*! Update Phases and Bezier Curve in different gait */
        this->Workers[this->flow]->run(&this->state_interior_);

        if(!this->model_StateEstimate->is_inited()){
            this->model_StateEstimate->init_state(this->state_interior_);
        }else{
            Update_StateEstimate();
        }
        /*! Convex MPC -> Calculate contact force */
        //this->state_interior_.foot_contact_force = this->mpc_solver.Calculate_contact_force(this->state_interior_);
        SendCommand();
    }
}

/*void FSM::set_timer() {
    this->FSM_Timer_ = this->nh_.createTimer(ros::Duration(this->state_interior_.plan_dt), &FSM::loop, this);

}

void FSM::set_timer2(){
    FSM_Timer2_ = this->nh_.createTimer(ros::Duration(this->state_interior_.plan_dt), &FSM::Update_MPC, this);
}*/

void FSM::Update_MPC(double dt) {
    this->state_interior_.foot_contact_force = this->mpc_solver.Calculate_contact_force(this->state_interior_, dt);
}

void FSM::build_ScheduleTable(int Schedule, ...) {
    va_list arg_ptr;
    va_start(arg_ptr, Schedule);
    while(Schedule != quad::END){
        switch (Schedule) {
            case quad::STAND:{
                StandWorker* tmp_Worker = new StandWorker(this->nh_);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::WALK:{
                float velocity = va_arg(arg_ptr, double);
                WalkWorker* tmp_Worker = new WalkWorker(this->nh_, velocity);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::TROT:{
                TrotWorker* tmp_Worker = new TrotWorker(this->nh_);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::PACE:{
                PaceWorker* tmp_Worker = new PaceWorker(this->nh_);
                this->Workers.push_back((StateWorker *)tmp_Worker);
                break;
            }
            case quad::GALLOP:{
                GallopWorker* tmp_Worker = new GallopWorker(this->nh_);
                this->Workers.push_back((StateWorker* )tmp_Worker);
                break;
            }
            default:
                ROS_ERROR("Wrong type of Schedule Table");
                exit(0);
                break;
        }
        Schedule = va_arg(arg_ptr, int);
    }
    return;
}

void FSM::Update_LegController(){
    /*! Update feedback */
    for (int foot = 0; foot < 4; foot++) {
        for (int joint = 0; joint < 3; joint++) {
            this->model_LegController->leg_data_[foot].q(joint) = this->state_interior_.joint_position(foot*3+joint);
            this->model_LegController->leg_data_[foot].qd(joint) = this->state_interior_.joint_velocity(foot*3+joint);
        }
    }

    /*! Update foot data, use Kinematics model */
    this->model_LegController->UpdateData();
    for (int foot = 0; foot < 4; foot++) {
        this->state_interior_.foot_p_robot.block<3, 1>(0, foot) =
                this->model_LegController->leg_data_[foot].p;
        this->state_interior_.foot_v_robot.block<3, 1>(0, foot) =
                this->model_LegController->leg_data_[foot].v;
        this->state_interior_.foot_q.block<3, 1>(0, foot) =
                this->model_LegController->leg_data_[foot].q;
        this->state_interior_.foot_qd.block<3, 1>(0, foot) =
                this->model_LegController->leg_data_[foot].qd;
        this->state_interior_.foot_jacobian.block<3, 3>(foot*3, foot*3) =
                this->model_LegController->leg_data_[foot].J;
        this->state_interior_.foot_jacobian_inv.block<3, 3>(foot*3, foot*3) =
                this->model_LegController->leg_data_[foot].J_inv;

        /*! add robot state to get foot position and velocity in world frame*/
        this->state_interior_.foot_p_abs.block<3, 1>(0, foot) =
                this->state_interior_.rotate_matrix
                        * this->state_interior_.foot_p_robot.block<3, 1>(0, foot);
        this->state_interior_.foot_p.block<3, 1>(0, foot) =
                this->state_interior_.foot_p_abs.block<3, 1>(0, foot)
                        + this->state_interior_.cur_position;
        this->state_interior_.foot_v_abs.block<3, 1>(0, foot) =
                this->state_interior_.rotate_matrix
                        * this->state_interior_.foot_v_robot.block<3, 1>(0, foot);
        this->state_interior_.foot_v.block<3, 1>(0, foot) =
                this->state_interior_.foot_v_abs.block<3, 1>(0, foot)
                        + this->state_interior_.cur_vel;

    }
}

void FSM::Update_StateEstimate() {
    /*! Linear KF Filter */
/*
    this->model_StateEstimate->LinearKF(this->state_interior_.foot_p_robot, this->state_interior_.foot_v_robot,
                                        this->state_interior_.phase_variable);

    this->state_interior_.estimate_position = this->model_StateEstimate->getPosition();
    this->state_interior_.estimate_vel = this->model_StateEstimate->getVelocity();
*/
    this->model_StateEstimate->update_estimation(this->state_interior_, this->state_interior_.plan_dt);

    this->state_interior_.cur_vel_b = this->state_interior_.rotate_matrix_z.transpose() * this->state_interior_.cur_vel;
    this->state_interior_.command_vel_b = this->state_interior_.rotate_matrix_z.transpose() * this->state_interior_.command_vel;


    /*! Update ros msg*/
    this->state_info_.cur_state.position.x = this->state_interior_.cur_position.x();
    this->state_info_.cur_state.position.y = this->state_interior_.cur_position.y();
    this->state_info_.cur_state.position.z = this->state_interior_.cur_position.z();

    this->state_info_.w_twist.linear.x = this->state_interior_.cur_vel.x();
    this->state_info_.w_twist.linear.y = this->state_interior_.cur_vel.y();
    this->state_info_.w_twist.linear.z = this->state_interior_.cur_vel.z();

    /*! Publish State Information */
    this->position_pub_.publish(this->state_info_.cur_state);
    this->velocity_pub_.publish(this->state_info_.w_twist.linear);
    this->euler_pub_.publish(this->state_info_.rpy_angle);
}

void FSM::SendCommand(){
    /*! Calculate torques */
    this->state_interior_.joint_torques = this->mpc_solver.Calculate_joint_torques(this->state_interior_);

    // send control cmd to robot via ros topic
    unitree_legged_msgs::LowCmd low_cmd;
    for (int i = 0; i < 4; i++) {
        if(0){
            /*! Position Control Without MPC */
                low_cmd.motorCmd[3*i].mode = 0x0A;
                low_cmd.motorCmd[3*i].q = 0;
                low_cmd.motorCmd[3*i].dq = 0;
                low_cmd.motorCmd[3*i].Kp = 70;
                low_cmd.motorCmd[3*i].Kd = 3;
                low_cmd.motorCmd[3*i].tau = 0;

                low_cmd.motorCmd[3*i+1].mode = 0x0A;
                low_cmd.motorCmd[3*i+1].q = this->state_interior_.foot_qDes(1, i);
                low_cmd.motorCmd[3*i+1].dq = 0;
                low_cmd.motorCmd[3*i+1].Kp = 180;
                low_cmd.motorCmd[3*i+1].Kd = 8;
                low_cmd.motorCmd[3*i+1].tau = 0;

                low_cmd.motorCmd[3*i+2].mode = 0x0A;
                low_cmd.motorCmd[3*i+2].q = this->state_interior_.foot_qDes(2, i);
                low_cmd.motorCmd[3*i+2].dq = 0;
                low_cmd.motorCmd[3*i+2].Kp = 300;
                low_cmd.motorCmd[3*i+2].Kd = 15;
                low_cmd.motorCmd[3*i+2].tau = 0;


        }
        else {
            /*! MPC Torques Control */
            low_cmd.motorCmd[3 * i].mode = 0x0A;
            low_cmd.motorCmd[3 * i].q = 0;
            low_cmd.motorCmd[3 * i].dq = 0;
            low_cmd.motorCmd[3 * i].Kp = 0;
            low_cmd.motorCmd[3 * i].Kd = 0;
            low_cmd.motorCmd[3 * i].tau = this->state_interior_.joint_torques(3*i, 0);

            low_cmd.motorCmd[3 * i + 1].mode = 0x0A;
            low_cmd.motorCmd[3 * i + 1].q = 0;
            low_cmd.motorCmd[3 * i + 1].dq = 0;
            low_cmd.motorCmd[3 * i + 1].Kp = 0;
            low_cmd.motorCmd[3 * i + 1].Kd = 0;
            low_cmd.motorCmd[3 * i + 1].tau = this->state_interior_.joint_torques(3*i+1, 0);

            low_cmd.motorCmd[3 * i + 2].mode = 0x0A;
            low_cmd.motorCmd[3 * i + 2].q = 0;
            low_cmd.motorCmd[3 * i + 2].dq = 0;
            low_cmd.motorCmd[3 * i + 2].Kp = 0;
            low_cmd.motorCmd[3 * i + 2].Kd = 0;
            low_cmd.motorCmd[3 * i + 2].tau = this->state_interior_.joint_torques(3*i+2, 0);
        }
    }

    for (int i = 0; i < 12; i++) {
        pub_joint_cmd[i].publish(low_cmd.motorCmd[i]);
    }
}

void FSM::MotorCallback(const std_msgs::Float64MultiArray &msg) {

}

void FSM::IMUCallback(const sensor_msgs::Imu &msg) {
    /*! Update ros msg */
    this->state_info_.cur_state.orientation = msg.orientation;
    this->state_info_.b_twist.angular = msg.angular_velocity;
    this->state_info_.b_linear_acceleration = msg.linear_acceleration;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(this->state_info_.cur_state.orientation, quat);
    Eigen::Quaterniond temp_quat;
    temp_quat.x() = this->quat_x.CalculateAverage(this->state_info_.cur_state.orientation.x);
    temp_quat.y() = this->quat_y.CalculateAverage(this->state_info_.cur_state.orientation.y);
    temp_quat.z() = this->quat_z.CalculateAverage(this->state_info_.cur_state.orientation.z);
    temp_quat.w() = this->quat_w.CalculateAverage(this->state_info_.cur_state.orientation.w);

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    this->state_info_.rpy_angle.x = roll;
    this->state_info_.rpy_angle.y = pitch;
    this->state_info_.rpy_angle.z = yaw;


    /*! Update interior msg */
    this->state_interior_.b_angle_vel.x() = gyro_x.CalculateAverage(msg.angular_velocity.x);
    this->state_interior_.b_angle_vel.y() = gyro_y.CalculateAverage(msg.angular_velocity.y);
    this->state_interior_.b_angle_vel.z() = gyro_z.CalculateAverage(msg.angular_velocity.z);
    this->state_interior_.w_angle_vel = this->state_interior_.rotate_matrix * this->state_interior_.b_angle_vel;

    this->state_interior_.b_acc.x() = acc_x.CalculateAverage(msg.linear_acceleration.x);
    this->state_interior_.b_acc.y() = acc_y.CalculateAverage(msg.linear_acceleration.y);
    this->state_interior_.b_acc.z() = acc_z.CalculateAverage(msg.linear_acceleration.z);
    this->state_interior_.cur_rpy.x() = roll;
    this->state_interior_.cur_rpy.y() = pitch;
    this->state_interior_.cur_rpy.z() = yaw;

    this->state_interior_.rotate_matrix_z = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    this->state_interior_.quaternion = temp_quat;
    this->state_interior_.rotate_matrix = this->state_interior_.quaternion.toRotationMatrix();

}

void FSM::JoyCallback(const sensor_msgs::Joy &msg) {
    this->state_interior_.command_vel.x() = msg.axes[4] * JOY_CMD_VELX_MAX;
    this->state_interior_.command_vel.y() = msg.axes[3] * JOY_CMD_VELY_MAX;
    this->state_interior_.command_vel.z() = msg.axes[1] * JOY_CMD_BODY_HEIGHT_VEL;

    this->state_interior_.command_position += this->state_interior_.command_vel * this->state_interior_.plan_dt;
    if(this->state_interior_.command_position.z() > JOY_CMD_BODY_HEIGHT_MAX){
        this->state_interior_.command_position.z() = JOY_CMD_BODY_HEIGHT_MAX;
    }
    if(this->state_interior_.command_position.z() < JOY_CMD_BODY_HEIGHT_MIN){
        this->state_interior_.command_position.z() = JOY_CMD_BODY_HEIGHT_MIN;
    }

    if(this->state_interior_.command_vel.segment<2>(0).norm() > 0.05){
        this->state_interior_.command_position.segment<2>(0) =
                this->state_interior_.cur_position.segment<2>(0);
    }

    this->state_interior_.command_angle_vel.z() = msg.axes[0] * JOY_CMD_YAW_MAX;

    this->state_interior_.command_rpy += this->state_interior_.command_angle_vel * this->state_interior_.plan_dt;

    if(msg.buttons[0] == 1){
        this->state_interior_.gait_type = quad::TROT;
    }

}

// FL
void FSM::FL_hip_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[0] = joint_state.q;
    this->state_interior_.joint_velocity[0] = joint_state.dq;
}
void FSM::FL_thigh_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[1] = joint_state.q;
    this->state_interior_.joint_velocity[1] = joint_state.dq;
}
void FSM::FL_calf_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[2] = joint_state.q;
    this->state_interior_.joint_velocity[2] = joint_state.dq;
}

// FR
void FSM::FR_hip_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[3] = joint_state.q;
    this->state_interior_.joint_velocity[3] = joint_state.dq;
}
void FSM::FR_thigh_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[4] = joint_state.q;
    this->state_interior_.joint_velocity[4] = joint_state.dq;
}
void FSM::FR_calf_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[5] = joint_state.q;
    this->state_interior_.joint_velocity[5] = joint_state.dq;
}

// RL
void FSM::RL_hip_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[6] = joint_state.q;
    this->state_interior_.joint_velocity[6] = joint_state.dq;
}
void FSM::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[7] = joint_state.q;
    this->state_interior_.joint_velocity[7] = joint_state.dq;
}
void FSM::RL_calf_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[8] = joint_state.q;
    this->state_interior_.joint_velocity[8] = joint_state.dq;
}

// RR
void FSM::RR_hip_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[9] = joint_state.q;
    this->state_interior_.joint_velocity[9] = joint_state.dq;
}
void FSM::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[10] = joint_state.q;
    this->state_interior_.joint_velocity[10] = joint_state.dq;
}
void FSM::RR_calf_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[11] = joint_state.q;
    this->state_interior_.joint_velocity[11] = joint_state.dq;
}


// foot contact force
void FSM::FL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    this->state_interior_.foot_force[0] = force.wrench.force.z;
}

void FSM::FR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    this->state_interior_.foot_force[1] = force.wrench.force.z;
}

void FSM::RL_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    this->state_interior_.foot_force[2] = force.wrench.force.z;
}

void FSM::RR_foot_contact_callback(const geometry_msgs::WrenchStamped &force) {
    this->state_interior_.foot_force[3] = force.wrench.force.z;
}



#endif
