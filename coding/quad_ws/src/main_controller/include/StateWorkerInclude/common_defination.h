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
            robot_mass = 13.0;
            inertia_tensor << 0.0158533, 0.0, 0.0,
                    0.0, 0.0377999, 0.0,
                    0.0, 0.0, 0.0456542;

            gait_type = STAND;
            leg_control_type = 1;

            plan_dt = 0.01;

            command_vel.setZero();
            command_angle_vel.setZero();
            command_rpy.setZero();


            cur_position.setZero();
            cur_vel.setZero();
            b_angle_vel.setZero();
            w_angle_vel.setZero();
            cur_rpy.setZero();
            b_acc.setZero();

            quaternion.setIdentity();
            rotate_matrix.setZero();

            foot_contact_force.setZero();
            foot_forces_swing.setZero();
            foot_p.setZero();
            foot_p_bias.setZero();
            foot_p_robot.setZero();
            foot_p_abs.setZero();
            foot_v.setZero();
            foot_v_robot.setZero();
            foot_q.setZero();
            foot_qd.setZero();
            foot_jacobian.setZero();
            foot_jacobian_inv.setZero();

            foot_pDes.setZero();
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
            q_weights << 80.0, 80.0, 1.0,
                    0.0, 0.0, 270.0,
                    1.0, 1.0, 20.0,
                    20.0, 20.0, 20.0,
                    0.0;
            r_weights << 1e-5, 1e-5, 1e-6,
                    1e-5, 1e-5, 1e-6,
                    1e-5, 1e-5, 1e-6,
                    1e-5, 1e-5, 1e-6;
            mpc_states.setZero();
            mpc_states_list.setZero();

            estimate_position.setZero();
            estimate_vel.setZero();

            joint_position.setZero();
            joint_velocity.setZero();
            joint_torques.setZero();

            /*! Init PD controller Parameters, Why ?*/
            double kp_foot_x = 150.0;
            double kp_foot_y = 150.0;
            double kp_foot_z = 200.0;
            double kd_foot_x = 0.0;
            double kd_foot_y = 0.0;
            double kd_foot_z = 0.0;
            kp_foot <<
                    kp_foot_x, kp_foot_x, kp_foot_x, kp_foot_x,
                    kp_foot_y, kp_foot_y, kp_foot_y, kp_foot_y,
                    kp_foot_z, kp_foot_z, kp_foot_z, kp_foot_z;
            kd_foot <<
                    kd_foot_x, kd_foot_x, kd_foot_x, kd_foot_x,
                    kd_foot_y, kd_foot_y, kd_foot_y, kd_foot_y,
                    kd_foot_z, kd_foot_z, kd_foot_z, kd_foot_z;
            km_foot = Eigen::Vector3d(0.1, 0.1, 0.04);

            kp_linear = Eigen::Vector3d(120.0, 120.0, 500.0);
            kd_linear = Eigen::Vector3d(70.0, 70.0, 120.0);
            kp_angular = Eigen::Vector3d(250.0, 35.0, 1.0);
            kd_angular = Eigen::Vector3d(1.5, 1.5, 30.0);
/*            kp_linear = Eigen::Vector3d(1.0, 1.0, 1.0);
            kd_linear = Eigen::Vector3d(0.0, 0.0, 0.0);
            kp_angular = Eigen::Vector3d(0.0, 0.0, 0.0);
            kd_angular = Eigen::Vector3d(0.0, 0.0, 0.0);*/

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
                foot_p_bias.block<3, 1>(0, foot).z() = -0.345;
            }
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*! interior params */
        double robot_mass;
        Eigen::Matrix3d inertia_tensor;

        /*! Gait Type*/
        STATE_TYPE gait_type;
        int leg_control_type; // 0->Stand/QP ; 1->other gait /MPC

        /*! Control Frequency */
        double plan_dt;

        /*! Command from Joy */
        Eigen::Vector3d command_vel;
        Eigen::Vector3d command_angle_vel;
        Eigen::Vector3d command_rpy;

        /*! Current State */
        Eigen::Vector3d cur_position;
        Eigen::Vector3d cur_vel;
        Eigen::Vector3d b_angle_vel; // in robot frame
        Eigen::Vector3d w_angle_vel; // in world frame
        Eigen::Vector3d cur_rpy;

        Eigen::Vector3d b_acc;       // in robot frame

        /*! Kinematic Parameters */
        Eigen::Quaterniond quaternion;
        Eigen::Matrix3d rotate_matrix;

        /*! Leg Controller Parameters */
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_swing;
        Eigen::Matrix<double, 3, NUM_LEG> foot_contact_force;
        // Data
        Eigen::Matrix<double, 3, NUM_LEG> foot_p;   // in world frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_p_robot;   // in robot frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_p_abs; // in a frame which centered at the robot frame's origin but parallels to the world frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_p_bias;
        Eigen::Matrix<double, 3, NUM_LEG> foot_v;
        Eigen::Matrix<double, 3, NUM_LEG> foot_v_robot;
        Eigen::Matrix<double, 3, NUM_LEG> foot_q;
        Eigen::Matrix<double, 3, NUM_LEG> foot_qd;
        Eigen::Matrix<double, 12, 12> foot_jacobian;
        Eigen::Matrix<double, 12, 12> foot_jacobian_inv;
        // Command in robot frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_pDes;
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
