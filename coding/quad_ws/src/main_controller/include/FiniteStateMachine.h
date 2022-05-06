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
    ros::Timer FSM_Timer_;
    /*! Publish Interior msg */
    ros::Publisher position_pub_, velocity_pub_, euler_pub_;
    /*! Publish Control msg */
    ros::Publisher angle_gazebo_pub_, angle_real_pub_;
    ros::Publisher pub_joint_cmd[12];

    ros::Subscriber motor_sub_, imu_sub_, joy_sub_;
    ros::Subscriber sub_joint_msg[12], sub_foot_contact_msg[4];

public:
    ros::NodeHandle nh_;
    std::vector<StateWorker*> Workers;
    int flow = 0;

    quad::STATE_INFO state_info_;

    STATE_INTERIOR state_interior_ ;

    /*! Class for control */
    StateEstimate* model_StateEstimate = new StateEstimate();
    A1BasicEKF* model_A1Estimate = new A1BasicEKF();
    LegController* model_LegController = new LegController();
    Solver mpc_solver;

    FSM(ros::NodeHandle &nh);
    ~FSM();
    void loop(const ros::TimerEvent &);
    void set_timer();
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
    this->pub_joint_cmd[6] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_hip_controller/command", 1);
    this->pub_joint_cmd[7] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_thigh_controller/command", 1);
    this->pub_joint_cmd[8] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RR_calf_controller/command", 1);
    this->pub_joint_cmd[9] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_hip_controller/command", 1);
    this->pub_joint_cmd[10] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_thigh_controller/command", 1);
    this->pub_joint_cmd[11] = this->nh_.advertise<unitree_legged_msgs::MotorCmd>("/a1_gazebo/RL_calf_controller/command", 1);

    this->motor_sub_ = this->nh_.subscribe("/quad/motor_info", 2, &FSM::MotorCallback ,this);
    this->imu_sub_ = this->nh_.subscribe("/body_imu", 2, &FSM::IMUCallback, this);
    this->joy_sub_ = this->nh_.subscribe("/joy", 2, &FSM::JoyCallback, this);

    this->sub_joint_msg[0] = this->nh_.subscribe("/a1_gazebo/FL_hip_controller/state", 2, &FSM::FL_hip_state_callback, this);
    this->sub_joint_msg[1] = this->nh_.subscribe("/a1_gazebo/FL_thigh_controller/state", 2, &FSM::FL_thigh_state_callback, this);
    this->sub_joint_msg[2] = this->nh_.subscribe("/a1_gazebo/FL_calf_controller/state", 2, &FSM::FL_calf_state_callback, this);
    this->sub_joint_msg[3] = this->nh_.subscribe("/a1_gazebo/FR_hip_controller/state", 2, &FSM::FR_hip_state_callback, this);
    this->sub_joint_msg[4] = this->nh_.subscribe("/a1_gazebo/FR_thigh_controller/state", 2, &FSM::FR_thigh_state_callback, this);
    this->sub_joint_msg[5] = this->nh_.subscribe("/a1_gazebo/FR_calf_controller/state", 2, &FSM::FR_calf_state_callback, this);
    this->sub_joint_msg[6] = this->nh_.subscribe("/a1_gazebo/RR_hip_controller/state", 2, &FSM::RR_hip_state_callback, this);
    this->sub_joint_msg[7] = this->nh_.subscribe("/a1_gazebo/RR_thigh_controller/state", 2, &FSM::RR_thigh_state_callback, this);
    this->sub_joint_msg[8] = this->nh_.subscribe("/a1_gazebo/RR_calf_controller/state", 2, &FSM::RR_calf_state_callback, this);
    this->sub_joint_msg[9] = this->nh_.subscribe("/a1_gazebo/RL_hip_controller/state", 2, &FSM::RL_hip_state_callback, this);
    this->sub_joint_msg[10] = this->nh_.subscribe("/a1_gazebo/RL_thigh_controller/state", 2, &FSM::RL_thigh_state_callback, this);
    this->sub_joint_msg[11] = this->nh_.subscribe("/a1_gazebo/RL_calf_controller/state", 2, &FSM::RL_calf_state_callback, this);


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
}

FSM::~FSM() {
    // release
    for(auto each:this->Workers){
        delete each;
    }
    delete this->model_StateEstimate;
    delete this->model_A1Estimate;
    delete this->model_LegController;
}

void FSM::loop(const ros::TimerEvent &) {
    /*! running the schedule table */
    if(this->Workers[this->flow]->is_finished()){
        if(this->state_interior_.gait_type == quad::STAND){
            /*! Update Stand Interior info */
            this->state_interior_.cur_position << 0, 0, 0.345;
            this->state_interior_.cur_vel << 0, 0, 0;
            this->model_LegController->FirstUpdateData(&this->state_interior_);
            //this->model_A1Estimate->init_state(this->state_interior_);
        }
        this->flow++;
        if(this->flow == this->Workers.size()){
            ROS_INFO("Finish ScheduleTable");
            exit(0);
        }
    }
    else{
        if(this->state_interior_.gait_type != quad::STAND){
            /*! Update data for state estimate */
            this->model_StateEstimate->UpdateOrientationData(this->state_interior_);
        }
        /*! Update Phases and Bezier Curve in different gait */
        this->Workers[this->flow]->run(&this->state_interior_);
        if(this->state_interior_.gait_type != quad::STAND){
            /*! Update Leg Control Data and Command */
            Update_LegController();
            Update_StateEstimate();
            //this->mpc_solver.update_plan(this->state_interior_, this->state_interior_.plan_dt);
            //this->mpc_solver.generate_swing_legs_ctrl(this->state_interior_, this->state_interior_.plan_dt);
            /*! Convex MPC -> Calculate contact force */
            this->state_interior_.foot_contact_force = this->mpc_solver.Calculate_contact_force(this->state_interior_);
            //std::cout << this->state_interior_.foot_contact_force << std::endl;
            SendCommand();
        }
    }
}

void FSM::set_timer() {
    this->FSM_Timer_ = this->nh_.createTimer(ros::Duration(0.01), &FSM::loop, this);

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
    for (int foot = 0; foot < 4; foot++) {
        this->model_LegController->leg_command_[foot].pDes =
                this->state_interior_.foot_pDes.block<3, 1>(0, foot);
        this->model_LegController->leg_command_[foot].vDes =
                this->state_interior_.foot_vDes.block<3, 1>(0, foot);
    }
    this->model_LegController->UpdateCommand();
    for (int foot = 0; foot < 4; foot++) {
        this->state_interior_.foot_qDes.block<3, 1>(0, foot) =
                this->model_LegController->leg_command_[foot].qDes;
        this->state_interior_.foot_qdDes.block<3, 1>(0, foot) =
                this->model_LegController->leg_command_[foot].qdDes;
    }
    this->model_LegController->UpdateData();
    /*! Update foot */
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

        /*! Update Params that control swing Legs */
        this->state_interior_.foot_forces_swing.block<3, 1>(0, foot) =
                (this->state_interior_.foot_pDes.block<3, 1>(0, foot)
                 -this->state_interior_.foot_p_robot.block<3, 1>(0, foot)).cwiseProduct(this->state_interior_.kp_foot.block<3, 1>(0, foot))
                +(this->state_interior_.foot_vDes.block<3, 1>(0, foot)
                 -this->state_interior_.foot_v_robot.block<3, 1>(0, foot)).cwiseProduct(this->state_interior_.kd_foot.block<3, 1>(0, foot));

    }

}

void FSM::Update_StateEstimate() {
    /*! Linear KF Filter */
    this->model_StateEstimate->LinearKF(this->state_interior_.foot_p_robot, this->state_interior_.foot_v_robot,
                                        this->state_interior_.phase_variable);

    this->state_interior_.estimate_position = this->model_StateEstimate->getPosition();
    this->state_interior_.estimate_position.z() = 0.345;
    this->state_interior_.estimate_vel = this->model_StateEstimate->getVelocity();

    //this->model_A1Estimate->update_estimation(this->state_interior_, this->state_interior_.plan_dt);

    /*! Update interior msg */
    this->state_interior_.cur_position = this->state_interior_.estimate_position;
    this->state_interior_.cur_vel = this->state_interior_.estimate_vel;

    /*! Update ros msg*/
    this->state_info_.cur_state.position.x = this->state_interior_.cur_position.x();
    this->state_info_.cur_state.position.y = this->state_interior_.cur_position.y();
    this->state_info_.cur_state.position.z = this->state_interior_.cur_position.z();

    this->state_info_.w_twist.linear.x = this->state_interior_.cur_vel.x();
    this->state_info_.w_twist.linear.y = this->state_interior_.cur_vel.y();
    this->state_info_.w_twist.linear.z = this->state_interior_.cur_vel.z();

    this->state_info_.b_twist.linear.x = this->model_StateEstimate->getVelocity_Body().x();
    this->state_info_.b_twist.linear.y = this->model_StateEstimate->getVelocity_Body().y();
    this->state_info_.b_twist.linear.z = this->model_StateEstimate->getVelocity_Body().z();


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
            low_cmd.motorCmd[3 * i].q = this->state_interior_.joint_torques(3*i, 0);
            low_cmd.motorCmd[3 * i].dq = 0;
            low_cmd.motorCmd[3 * i].Kp = 0;
            low_cmd.motorCmd[3 * i].Kd = 0;
            low_cmd.motorCmd[3 * i].tau = 0;

            low_cmd.motorCmd[3 * i + 1].mode = 0x0A;
            low_cmd.motorCmd[3 * i + 1].q = this->state_interior_.foot_qDes(1, i);
            low_cmd.motorCmd[3 * i + 1].dq = 0;
            low_cmd.motorCmd[3 * i + 1].Kp = 0;
            low_cmd.motorCmd[3 * i + 1].Kd = 0;
            low_cmd.motorCmd[3 * i + 1].tau = this->state_interior_.joint_torques(3*i+1, 0);

            low_cmd.motorCmd[3 * i + 2].mode = 0x0A;
            low_cmd.motorCmd[3 * i + 2].q = this->state_interior_.foot_qDes(2, i);
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
    temp_quat.x() = this->state_info_.cur_state.orientation.x;
    temp_quat.y() = this->state_info_.cur_state.orientation.y;
    temp_quat.z() = this->state_info_.cur_state.orientation.z;
    temp_quat.w() = this->state_info_.cur_state.orientation.w;

    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    this->state_info_.rpy_angle.x = roll;
    this->state_info_.rpy_angle.y = pitch;
    this->state_info_.rpy_angle.z = yaw;

    /*! Update interior msg */
    this->state_interior_.b_angle_vel.x() = msg.angular_velocity.x;
    this->state_interior_.b_angle_vel.y() = msg.angular_velocity.y;
    this->state_interior_.b_angle_vel.z() = msg.angular_velocity.z;
    this->state_interior_.w_angle_vel = this->state_interior_.rotate_matrix * this->state_interior_.b_angle_vel;

    this->state_interior_.b_acc.x() = msg.linear_acceleration.x;
    this->state_interior_.b_acc.y() = msg.linear_acceleration.y;
    this->state_interior_.b_acc.z() = msg.linear_acceleration.z;
    this->state_interior_.cur_rpy.x() = roll;
    this->state_interior_.cur_rpy.y() = pitch;
    this->state_interior_.cur_rpy.z() = yaw;

    this->state_interior_.quaternion = temp_quat;
    this->state_interior_.rotate_matrix = this->state_interior_.quaternion.toRotationMatrix();

}

void FSM::JoyCallback(const sensor_msgs::Joy &msg) {
    this->state_interior_.command_vel.x() = msg.axes[1] * 0.2;
    this->state_interior_.command_rpy << 0,0,0;
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

// RR
void FSM::RR_hip_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[6] = joint_state.q;
    this->state_interior_.joint_velocity[6] = joint_state.dq;
}
void FSM::RR_thigh_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[7] = joint_state.q;
    this->state_interior_.joint_velocity[7] = joint_state.dq;
}
void FSM::RR_calf_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[8] = joint_state.q;
    this->state_interior_.joint_velocity[8] = joint_state.dq;
}

// RL
void FSM::RL_hip_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[9] = joint_state.q;
    this->state_interior_.joint_velocity[9] = joint_state.dq;
}
void FSM::RL_thigh_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
    this->state_interior_.joint_position[10] = joint_state.q;
    this->state_interior_.joint_velocity[10] = joint_state.dq;
}
void FSM::RL_calf_state_callback(const unitree_legged_msgs::MotorState &joint_state) {
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
