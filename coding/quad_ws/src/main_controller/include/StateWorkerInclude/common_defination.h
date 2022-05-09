//
// Created by demphi on 2021/10/1.
//

#ifndef _COMMON_DEFINATION_
#define _COMMON_DEFINATION_

#include "common_parameter.h"
#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

namespace quad{
    /*! State Mode */
    enum STATE_TYPE{
        INIT,
        STAND,
        WALK,
        TROT,
        PACE,
        GALLOP,
        END
    };

    /*! state info, Ros */
    struct STATE_INFO{
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*! current state */
        geometry_msgs::Pose cur_state;

        /*! imu msg */
        geometry_msgs::Twist b_twist;
        geometry_msgs::Twist w_twist;
        geometry_msgs::Vector3 b_linear_acceleration;
        geometry_msgs::Vector3 rpy_angle;

        /*! Control msg */
        std_msgs::Float64MultiArray angle_gazebo_data;
        std_msgs::Float64MultiArray angle_real_data;

        /*! feedback info */
        std_msgs::Float64MultiArray position_feedback_info;
        std_msgs::Float64MultiArray velocity_feedback_info;
        std_msgs::Float64MultiArray current_feedback_info;
    };
}

using namespace quad;
/*! state info, Matrix*/
class STATE_INTERIOR{
    public:
        STATE_INTERIOR(){
            Reset();
            InitParams();
        }

        void Reset(){
            /*! For test parameters */
            counter_per_gait = 240;
            counter_per_swing = 120;
            counter = 0;
            gait_counter.setZero();
            foot_pos_start.setZero();
            foot_pos_rel_last_time.setZero();
            foot_pos_target_last_time.setZero();
            gait_counter_reset();
//            gait_counter_speed << 1.5, 1.5, 1.5, 1.5 ;
            gait_counter_speed << 2, 2, 2, 2 ;



            robot_mass = 12.0;
            inertia_tensor << 0.0158533, 0.0, 0.0,
                    0.0, 0.0377999, 0.0,
                    0.0, 0.0, 0.0456542;

            gait_type = STAND;
            leg_control_type = 1;

            plan_dt = 0.0025;

            command_position << 0,0,0.32;
            command_vel.setZero();
            command_vel_b.setZero();
            command_angle_vel.setZero();
            command_rpy.setZero();


            cur_position.setZero();
            cur_vel.setZero();
            cur_vel_b.setZero();
            b_angle_vel.setZero();
            w_angle_vel.setZero();
            cur_rpy.setZero();
            b_acc.setZero();

            quaternion.setIdentity();
            rotate_matrix.setZero();
            rotate_matrix_z.setZero();


            foot_force.setZero();
            foot_contact_force.setZero();
            foot_forces_swing.setZero();
            foot_p.setZero();
            foot_p_bias.setZero();
            foot_p_robot.setZero();
            foot_p_abs.setZero();
            foot_v.setZero();
            foot_v_robot.setZero();
            foot_v_abs.setZero();
            foot_q.setZero();
            foot_qd.setZero();
            foot_jacobian.setZero();
            foot_jacobian_inv.setZero();

            foot_pDes.setZero();
            foot_pDes_robot.setZero();
            foot_pDes_abs.setZero();
            foot_vDes.setZero();
            foot_qDes.setZero();
            foot_qdDes.setZero();

            for (int i = 0; i < NUM_LEG; i++) {
                contacts[i] = 0;
                plan_contacts[i] = false;
                early_contacts[i] = false;
                estimate_contacts[i] = false;
            }
            phase_variable.setZero();

            q_weights.resize(MPC_STATE_DIM);
            r_weights.resize(12);
            q_weights << 20.0, 10.0, 1.0,
                    0.0, 0.0, 420.0,
                    0.05, 0.05, 0.05,
                    30.0, 30.0, 10.0,
                    0.0;
            r_weights << 1e-7, 1e-7, 1e-7,
                    1e-7, 1e-7, 1e-7,
                    1e-7, 1e-7, 1e-7,
                    1e-7, 1e-7, 1e-7;
            mpc_states.setZero();
            mpc_states_list.setZero();

            estimate_position.setZero();
            estimate_vel.setZero();

            joint_position.setZero();
            joint_velocity.setZero();
            joint_torques.setZero();

            /*! Init PD controller Parameters, Why ?*/
            double kp_foot_x = 200.0;
            double kp_foot_y = 200.0;
            double kp_foot_z = 150.0;
            double kd_foot_x = 10.0;
            double kd_foot_y = 10.0;
            double kd_foot_z = 5.0;
            kp_foot <<
                    kp_foot_x, kp_foot_x, kp_foot_x, kp_foot_x,
                    kp_foot_y, kp_foot_y, kp_foot_y, kp_foot_y,
                    kp_foot_z, kp_foot_z, kp_foot_z, kp_foot_z;
            kd_foot <<
                    kd_foot_x, kd_foot_x, kd_foot_x, kd_foot_x,
                    kd_foot_y, kd_foot_y, kd_foot_y, kd_foot_y,
                    kd_foot_z, kd_foot_z, kd_foot_z, kd_foot_z;
            km_foot = Eigen::Vector3d(0.1, 0.1, 0.1);

            kp_linear = Eigen::Vector3d(120.0, 120.0, 500.0);
            kd_linear = Eigen::Vector3d(70.0, 70.0, 120.0);
            kp_angular = Eigen::Vector3d(250.0, 35.0, 1.0);
            kd_angular = Eigen::Vector3d(1.5, 1.5, 30.0);

            torques_gravity << 0.80, 0, 0, -0.80, 0, 0, 0.80, 0, 0, -0.80, 0, 0;

        }

        void InitParams(){
            for (int foot = 0; foot < NUM_LEG; foot++) {
                if(foot == 0 || foot == 1){
                    foot_p_bias.block<3, 1>(0, foot).x() = X_OFFSET;
                }else{
                    foot_p_bias.block<3, 1>(0, foot).x() = -X_OFFSET;
                }
                if(foot == 0 || foot == 2){
                    foot_p_bias.block<3, 1>(0, foot).y() = Y_OFFSET;
                }else{
                    foot_p_bias.block<3, 1>(0, foot).y() = -Y_OFFSET;
                }
                foot_p_bias.block<3, 1>(0, foot).z() = -0.33;
            }
        }

        void gait_counter_reset() {
            gait_counter << 0, 120, 120, 0;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*! For test parameters */
        double counter_per_gait;
        double counter_per_swing;
        int counter;
        Eigen::Vector4d gait_counter;
        Eigen::Vector4d gait_counter_speed;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_start;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_rel_last_time;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target_last_time;


    /*! interior params */
        double robot_mass;
        Eigen::Matrix3d inertia_tensor;

        /*! Gait Type*/
        STATE_TYPE gait_type;
        int leg_control_type;

        /*! Control Frequency */
        double plan_dt;

        /*! Command from Joy */
        Eigen::Vector3d command_position;
        Eigen::Vector3d command_vel;
        Eigen::Vector3d command_vel_b;
        Eigen::Vector3d command_angle_vel;
        Eigen::Vector3d command_rpy;

        /*! Current State */
        Eigen::Vector3d cur_position;
        Eigen::Vector3d cur_vel;
        Eigen::Vector3d cur_vel_b;
        Eigen::Vector3d b_angle_vel; // in robot frame
        Eigen::Vector3d w_angle_vel; // in world frame
        Eigen::Vector3d cur_rpy;

        Eigen::Vector3d b_acc;       // in robot frame

        /*! Kinematic Parameters */
        Eigen::Quaterniond quaternion;
        Eigen::Matrix3d rotate_matrix;
        Eigen::Matrix3d rotate_matrix_z;

        /*! Leg Controller Parameters */
        Eigen::Vector4d foot_force;
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_swing;
        Eigen::Matrix<double, 3, NUM_LEG> foot_contact_force;
        // Data
        Eigen::Matrix<double, 3, NUM_LEG> foot_p;   // in world frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_p_robot;   // in robot frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_p_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_p_bias;
        Eigen::Matrix<double, 3, NUM_LEG> foot_v;
        Eigen::Matrix<double, 3, NUM_LEG> foot_v_robot;
        Eigen::Matrix<double, 3, NUM_LEG> foot_v_abs;
        Eigen::Matrix<double, 3, NUM_LEG> foot_q;
        Eigen::Matrix<double, 3, NUM_LEG> foot_qd;
        Eigen::Matrix<double, 12, 12> foot_jacobian;
        Eigen::Matrix<double, 12, 12> foot_jacobian_inv;
        // Command in robot frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_pDes;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pDes_robot;
        Eigen::Matrix<double, 3, NUM_LEG> foot_pDes_abs;
        Eigen::Matrix<double, 3, NUM_LEG> foot_vDes;
        Eigen::Matrix<double, 3, NUM_LEG> foot_qDes;
        Eigen::Matrix<double, 3, NUM_LEG> foot_qdDes;




        /*! Gait Phase Parameters */
        int contacts[NUM_LEG]; // True, if foot in contact
        bool plan_contacts[NUM_LEG];
        bool early_contacts[NUM_LEG];   // True, if foot hit objects during swing
        Eigen::Vector4d phase_variable;

        /*! MPC Parameters */
        Eigen::VectorXd q_weights;
        Eigen::VectorXd r_weights;
        Eigen::Matrix<double, MPC_STATE_DIM, 1> mpc_states;
        Eigen::Matrix<double, MPC_STATE_DIM*PLAN_HORIZON, 1> mpc_states_list;


        /*! State Estimate Parameters */
        bool estimate_contacts[NUM_LEG];
        Eigen::Vector3d estimate_position; // in world frame
        Eigen::Vector3d estimate_vel;   // in world frame

        /*! PD Parameters */
        Eigen::Matrix<double, NUM_DOF, 1>joint_position;
        Eigen::Matrix<double, NUM_DOF, 1>joint_velocity;
        Eigen::Matrix<double, NUM_DOF, 1>joint_torques;
        Eigen::Matrix<double, NUM_DOF, 1>torques_gravity;

        // Still have problems in some parameters
        double kp_lin_x;
        double kd_lin_x;
        double kf_lin_x;
        double kp_lin_y;
        double kd_lin_y;
        double kf_lin_y;

        Eigen::Matrix<double, 3, NUM_LEG> kp_foot;
        Eigen::Matrix<double, 3, NUM_LEG> kd_foot;
        Eigen::Matrix<double, 3, 1> km_foot;

        Eigen::Vector3d kp_linear;
        Eigen::Vector3d kd_linear;
        Eigen::Vector3d kp_angular;
        Eigen::Vector3d kd_angular;

    };


#endif 
