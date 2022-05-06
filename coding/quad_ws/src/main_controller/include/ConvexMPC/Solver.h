//
// Created by demphi on 2022/4/29.
//

#ifndef MAIN_CONTROLLER_SOLVER_H
#define MAIN_CONTROLLER_SOLVER_H

#include "reference/common_include.h"
#include "ConvexMPC.h"
#include "OsqpEigen/OsqpEigen.h"

using namespace quad;


class Solver{
public:
    BezierUtils bezierUtils[NUM_LEG];

    OsqpEigen::Solver solver;
    /*! QP Formation */
    Eigen::DiagonalMatrix<double, 6>Q;
    Eigen::Matrix<double, 3, NUM_LEG> last_force;
    double R;
    // ground friction coefficient
    double mu;
    double F_min;
    double F_max;
    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    //MPC does not start for the first 10 ticks to prevent uninitialized NAN goes into joint_torques
    int mpc_init_counter;

    Solver(){
        InitParams();
    }

    ~Solver(){

    }

    void InitParams(){
        ROS_INFO("[MPC SOLVER] Init QP Solver, MPC Parameters");
        Q.diagonal() << 1.0, 1.0, 1.0, 400.0, 400.0, 100.0;
        R = 1e-3;
        mu = 0.7;
        F_min = 0;
        F_max = 180;
        last_force.setZero();

        hessian.resize(3 * NUM_LEG, 3 * NUM_LEG);
        gradient.resize(3 * NUM_LEG);
        linearMatrix.resize(NUM_LEG + 4 * NUM_LEG, 3 * NUM_LEG);
        lowerBound.resize(NUM_LEG + 4 * NUM_LEG);
        lowerBound.setZero();
        upperBound.resize(NUM_LEG + 4 * NUM_LEG);
        upperBound.setZero();

        // init mpc skip counter
        mpc_init_counter = 0;
        // init constraints matrix
        for (int i = 0; i < NUM_LEG; i++) {
            // extract F_zi
            linearMatrix.insert(i, 2 + i * 3) = 1;
            // friction pyramid
            // 1. F_xi < uF_zi
            linearMatrix.insert(NUM_LEG + i * 4, i * 3) = 1;
            linearMatrix.insert(NUM_LEG + i * 4, 2 + i * 3) = -mu;
            lowerBound(NUM_LEG + i * 4) = -OsqpEigen::INFTY;
            // 2. F_xi > -uF_zi    ===> -F_xi -uF_zi < 0
            linearMatrix.insert(NUM_LEG + i * 4 + 1, i * 3) = -1;
            linearMatrix.insert(NUM_LEG + i * 4 + 1, 2 + i * 3) = -mu;
            lowerBound(NUM_LEG + i * 4 + 1) = -OsqpEigen::INFTY;
            // 3. F_yi < uF_zi
            linearMatrix.insert(NUM_LEG + i * 4 + 2, 1 + i * 3) = 1;
            linearMatrix.insert(NUM_LEG + i * 4 + 2, 2 + i * 3) = -mu;
            lowerBound(NUM_LEG + i * 4 + 2) = -OsqpEigen::INFTY;
            // 4. -F_yi > uF_zi
            linearMatrix.insert(NUM_LEG + i * 4 + 3, 1 + i * 3) = -1;
            linearMatrix.insert(NUM_LEG + i * 4 + 3, 2 + i * 3) = -mu;
            lowerBound(NUM_LEG + i * 4 + 3) = -OsqpEigen::INFTY;

        }
    }

    void update_plan(STATE_INTERIOR &state, double dt) {
        state.counter += 1;
        if (!state.movement_mode) {
            // movement_mode == 0, standstill with all feet in contact with ground
            for (bool &plan_contact: state.plan_contacts) plan_contact = true;
            state.gait_counter_reset();
        } else {
            // movement_mode == 1, walk
            for (int i = 0; i < NUM_LEG; ++i) {
                state.gait_counter(i) = state.gait_counter(i) + state.gait_counter_speed(i);
                state.gait_counter(i) = std::fmod(state.gait_counter(i), state.counter_per_gait);
                if (state.gait_counter(i) <= state.counter_per_swing) {
                    state.plan_contacts[i] = true;
                } else {
                    state.plan_contacts[i] = false;
                }
            }
        }

        // update foot plan: state.foot_pos_target_world
        Eigen::Vector3d lin_vel_world = state.cur_vel; // world frame linear velocity
        Eigen::Vector3d lin_vel_rel = state.rotate_matrix.transpose() * lin_vel_world; // robot body frame linear velocity

        // Raibert Heuristic, calculate foothold position
        state.foot_pDes = state.foot_p_bias;
        for (int i = 0; i < NUM_LEG; ++i) {
            double delta_x =
                    std::sqrt(std::abs(state.foot_p_bias(2)) / 9.8) * (lin_vel_rel(0) - state.command_vel(0)) +
                    ((state.counter_per_swing / state.gait_counter_speed(i)) * state.plan_dt) / 2.0 *
                    state.command_vel(0);
            double delta_y =
                    std::sqrt(std::abs(state.foot_p_bias(2)) / 9.8) * (lin_vel_rel(1) - state.command_vel(1)) +
                    ((state.counter_per_swing / state.gait_counter_speed(i)) * state.plan_dt) / 2.0 *
                    state.command_vel(1);

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

            state.foot_pos_target_rel(0, i) += delta_x;
            state.foot_pos_target_rel(1, i) += delta_y;

            state.foot_pos_target_abs.block<3, 1>(0, i) = state.rotate_matrix * state.foot_pos_target_rel.block<3, 1>(0, i);
            state.foot_pos_target_world.block<3, 1>(0, i) = state.foot_pos_target_abs.block<3, 1>(0, i) + state.cur_position;
        }
    }

    void generate_swing_legs_ctrl(STATE_INTERIOR &state, double dt) {
        state.joint_torques.setZero();

        // get current foot pos and target foot pose
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
        Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
        Eigen::Matrix<float, 1, NUM_LEG> spline_time;
        spline_time.setZero();
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
        foot_pos_target.setZero();
        Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;
        foot_vel_target.setZero();
        Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
        Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;

        // the foot force of swing foot and stance foot, both are in robot frame
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_swing;
        Eigen::Matrix<double, 3, NUM_LEG> foot_forces_grf;

        for (int i = 0; i < NUM_LEG; ++i) {
            foot_pos_cur.block<3, 1>(0, i) = state.rotate_matrix.transpose() * state.foot_p_abs.block<3, 1>(0, i);

            // from foot_pos_cur to foot_pos_final computes an intermediate point using BezierUtils
            if (state.gait_counter(i) <= state.counter_per_swing) {
                // stance foot
                spline_time(i) = 0.0;
                // in this case the foot should be stance
                // keep refreshing foot_pos_start in stance mode
                state.foot_pos_start.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);
            } else {
                // in this case the foot should be swing
                spline_time(i) =
                        float(state.gait_counter(i) - state.counter_per_swing) / float(state.counter_per_swing);
            }

            foot_pos_target.block<3, 1>(0, i) = bezierUtils[i].get_foot_pos_curve(spline_time(i),
                                                                                  state.foot_pos_start.block<3, 1>(0,
                                                                                                                   i),
                                                                                  state.foot_pos_target_rel.block<3, 1>(
                                                                                          0, i),
                                                                                  0.0);

            foot_vel_cur.block<3, 1>(0, i) =
                    (foot_pos_cur.block<3, 1>(0, i) - state.foot_pos_rel_last_time.block<3, 1>(0, i)) / dt;
            state.foot_pos_rel_last_time.block<3, 1>(0, i) = foot_pos_cur.block<3, 1>(0, i);

            foot_vel_target.block<3, 1>(0, i) =
                    (foot_pos_target.block<3, 1>(0, i) - state.foot_pos_target_last_time.block<3, 1>(0, i)) / dt;
            state.foot_pos_target_last_time.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i);

            foot_pos_error.block<3, 1>(0, i) = foot_pos_target.block<3, 1>(0, i) - foot_pos_cur.block<3, 1>(0, i);
            foot_vel_error.block<3, 1>(0, i) = foot_vel_target.block<3, 1>(0, i) - foot_vel_cur.block<3, 1>(0, i);
            foot_forces_swing.block<3, 1>(0, i) =
                    foot_pos_error.block<3, 1>(0, i).cwiseProduct(state.kp_foot.block<3, 1>(0, i)) +
                    foot_vel_error.block<3, 1>(0, i).cwiseProduct(state.kd_foot.block<3, 1>(0, i));
        }
        state.foot_pos_cur = foot_pos_cur;

        // detect early contact
        bool last_contacts[NUM_LEG];

        for (int i = 0; i < NUM_LEG; ++i) {
            if (state.gait_counter(i) <= state.counter_per_swing * 1.5) {
                state.early_contacts[i] = false;
            }
            if (!state.plan_contacts[i] &&
                (state.gait_counter(i) > state.counter_per_swing * 1.5) &&
                (state.foot_force(i) > FOOT_FORCE_LOW)) {
                state.early_contacts[i] = true;
            }

            // actual contact
            last_contacts[i] = state.contacts[i];
            state.contacts[i] = state.plan_contacts[i] || state.early_contacts[i];


        }
        state.foot_forces_swing = foot_forces_swing;
    }

    /*! Use contact force to Calculate joint torques */
    Eigen::Matrix<double, NUM_DOF, 1> Calculate_joint_torques(STATE_INTERIOR &stateInterior){
        Eigen::Matrix<double, NUM_DOF, 1> joint_torques;

        joint_torques.setZero();
        mpc_init_counter++;
        // for the first 10 ticks, just return zero torques.
        if(mpc_init_counter < 10){
            stateInterior.joint_torques = joint_torques;
        }else{
            // for each leg, if it is a swing leg (contact[i] is false), use foot_force_kin getting joint_torque
            // for each leg, if it is a stance leg (contact[i] is true), use foot_contact_force getting joint_torque
            for (int i = 0; i < NUM_LEG; i++) {
                Eigen::Matrix3d jac = stateInterior.foot_jacobian.block<3, 3>(3 * i, 3 * i);
                if (stateInterior.contacts[i] == 1) {
                    // stance
                    joint_torques.segment<3>(i * 3) = jac.transpose()
                                                      * -stateInterior.foot_contact_force.block<3, 1>(0, i);
                } else {
                    // swing
                    Eigen::Vector3d force_tgt = stateInterior.km_foot.cwiseProduct(
                            stateInterior.foot_forces_swing.block<3, 1>(0, i));
                    // X = A.lu().solve(b)
                    joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt); // LU分解, jac * tau = F
                    //joint_torques.segment<3>(i*3) = jac.transpose() * force_tgt;
                }
            }
            // add gravity compensation
            joint_torques += stateInterior.torques_gravity;

            // prevent nan
            for (int i = 0; i < 12; i++) {
                if (!isnan(joint_torques[i]))
                    return joint_torques;
            }
        }
    }

    /*! Contact force -> [fx fy fz] * 4 */
    Eigen::Matrix<double, 3, NUM_LEG> Calculate_contact_force(STATE_INTERIOR &stateInterior){
        Eigen::Matrix<double, 3, NUM_LEG> foot_contact_force;
        Eigen::Vector3d rpy_error = stateInterior.command_rpy - stateInterior.cur_rpy;
        // limit euler error to pi/2
        if (rpy_error(2) > 3.1415926 * 1.5) {
            rpy_error(2) = stateInterior.command_rpy(2) - 3.1415926 * 2 - stateInterior.cur_rpy(2);
        } else if (rpy_error(2) < -3.1415926 * 1.5) {
            rpy_error(2) = stateInterior.command_rpy(2) + 3.1415926 * 2 - stateInterior.cur_rpy(2);
        }
        if(stateInterior.leg_control_type == 0){
            // no use ???
        }
        else if (stateInterior.leg_control_type == 1){
            ConvexMPC mpc_solver = ConvexMPC(stateInterior.q_weights, stateInterior.r_weights);
            mpc_solver.Reset();

            // initialize the mpc state at the first time step
            // state.mpc_states.resize(13);
            stateInterior.mpc_states << stateInterior.cur_rpy.x(), stateInterior.cur_rpy.y(),stateInterior.cur_rpy.z(),
                stateInterior.cur_position.x(), stateInterior.cur_position.y(), stateInterior.cur_position.z(),
                stateInterior.w_angle_vel.x(), stateInterior.w_angle_vel.y(), stateInterior.w_angle_vel.z(),
                stateInterior.cur_vel.x(), stateInterior.cur_vel.y(), stateInterior.cur_vel.z(),
                -9.8;
            // this should be roughly close to the average dt of main controller
            double mpc_dt = 0.01;
            // if in gazebo
            mpc_dt = stateInterior.plan_dt;

            // initialize the desired mpc states trajectory
            for (int i = 0; i < PLAN_HORIZON; i++) {
                stateInterior.mpc_states_list.segment(i * 13, 13) <<
                    stateInterior.command_rpy.x(),
                    stateInterior.command_rpy.y(),
                    stateInterior.cur_rpy.z() + stateInterior.command_angle_vel.z() * mpc_dt * (i+1),
                    stateInterior.cur_position.x() + stateInterior.command_vel.x() * mpc_dt * (i+1),
                    stateInterior.cur_position.y() + stateInterior.command_vel.y() * mpc_dt * (i+1),
                    stateInterior.cur_position.z(), // desired position z = current position z
                    stateInterior.command_angle_vel.x(),
                    stateInterior.command_angle_vel.y(),
                    stateInterior.command_angle_vel.z(),
                    stateInterior.command_vel.x(),
                    stateInterior.command_vel.y(),
                    0, -9.8;
            }


            // a single A_mat_c is computed for the entire reference trajectory
            mpc_solver.Calculate_A_mat_c(stateInterior.cur_rpy);
            // for each point in the reference trajectory, an approximate B_c matrix is computed using desired values of euler angles and feet positions
            // from the reference trajectory and foot placement controller
            for (int i = 0; i < PLAN_HORIZON; i++) {

                // Calculate current B_mat_c Matrix
                mpc_solver.Calculate_B_mat_c(stateInterior.robot_mass,
                                             stateInterior.inertia_tensor,
                                             stateInterior.rotate_matrix,
                                             stateInterior.foot_p_abs);
                // state space Discretization
                mpc_solver.State_space_discrete(mpc_dt);

                // store current B_d Matrix
                mpc_solver.B_mat_d_list.block<13, 12>(i*13, 0) = mpc_solver.B_mat_d;
            }
            // Calculate QP Matrix
            mpc_solver.Calculate_qp(stateInterior);

            // Solve the QP Problem
            if(!solver.isInitialized()){
                // Setting
                solver.settings()->setVerbosity(false);
                solver.settings()->setWarmStart(true);
                // Set the initial data of the QP solver
                solver.data()->setNumberOfVariables(NUM_DOF * PLAN_HORIZON);
                solver.data()->setNumberOfConstraints(MPC_CONSTRAINT_DIM * PLAN_HORIZON);
                solver.data()->setLinearConstraintsMatrix(mpc_solver.linear_constraints);
                solver.data()->setHessianMatrix(mpc_solver.hessian);
                solver.data()->setGradient(mpc_solver.gradient);
                solver.data()->setLowerBound(mpc_solver.lb);
                solver.data()->setUpperBound(mpc_solver.ub);
                solver.initSolver();

            }else{
                solver.updateHessianMatrix(mpc_solver.hessian);
                solver.updateGradient(mpc_solver.gradient);
                solver.updateLowerBound(mpc_solver.lb);
                solver.updateUpperBound(mpc_solver.ub);
            }

            solver.solve();
            // Get Solution
            Eigen::VectorXd solution = solver.getSolution();

            if(solution[0] > 20000){
                std::cout << " Wrong contact force! " << std::endl;

/*                std::cout << " foot_position   " << std::endl;
                std::cout << stateInterior.foot_p_abs << std::endl;
                std::cout << stateInterior.foot_p << std::endl;
                std::cout << stateInterior.foot_p_robot << std::endl;
                std::cout << stateInterior.foot_pDes << std::endl;*/
            }
            for (int i = 0; i < NUM_LEG; i++) {
                if(!isnan(solution.segment<3>(i*3).norm())){
                    foot_contact_force.block<3, 1>(0, i) = stateInterior.rotate_matrix.transpose() * solution.segment<3>(i*3);
                }
            }
            std::cout << stateInterior.foot_contact_force << std::endl;
        }
    }

};

#endif 
