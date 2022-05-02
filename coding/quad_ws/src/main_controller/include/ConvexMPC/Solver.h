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
    OsqpEigen::Solver solver;
    /*! QP Formation */
    Eigen::DiagonalMatrix<double, 6>Q;
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


    /*! Use contact force to Calculate joint torques */
    Eigen::Matrix<double, NUM_DOF, 1> Calculate_joint_torques(STATE_INTERIOR &stateInterior){
        Eigen::Matrix<double, NUM_DOF, 1> joint_torques;
        joint_torques.setZero();
        mpc_init_counter++;
        // for the first 10 ticks, just return zero torques.
        if(mpc_init_counter < 0){
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
                    //joint_torques.segment<3>(i * 3) = jac.lu().solve(force_tgt); // LU分解, jac * tau = F
                    joint_torques.segment<3>(i*3) = jac.transpose() * force_tgt;
                }
            }
            // add gravity compensation
            joint_torques += stateInterior.torques_gravity;

            // prevent nan
            for (int i = 0; i < 12; i++) {
                if (!isnan(joint_torques[i]))
                    stateInterior.joint_torques[i] = joint_torques[i];
            }

            return joint_torques;
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


            // a single A_hat_c is computed for the entire reference trajectory
            mpc_solver.Calculate_A_hat_c(stateInterior.cur_rpy);
            // for each point in the reference trajectory, an approximate B_c matrix is computed using desired values of euler angles and feet positions
            // from the reference trajectory and foot placement controller
            for (int i = 0; i < PLAN_HORIZON; i++) {
                // Calculate current B_hat_c Matrix
                mpc_solver.Calculate_B_hat_c(stateInterior.robot_mass,
                                             stateInterior.inertia_tensor,
                                             stateInterior.rotate_matrix,
                                             stateInterior.foot_p_abs);
                // state space Discretization
                mpc_solver.State_space_discrete(mpc_dt);

                // store current B_d Matrix
                mpc_solver.B_hat_d_list.block<13, 12>(i*13, 0) = mpc_solver.B_hat_d;
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
            // Timer
            auto t1 = std::chrono::high_resolution_clock::now();
            solver.solve();
            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms_double_5 = t2 - t1;

            // Get Solution
            Eigen::VectorXd solution = solver.getSolution();

            for (int i = 0; i < NUM_LEG; i++) {
                if(!isnan(solution.segment<3>(i*3).norm())){
                    foot_contact_force.block<3, 1>(0, i) = stateInterior.rotate_matrix.transpose() * solution.segment<3>(i*3);
                }
            }
        }
        return foot_contact_force;
    }

private:

};

#endif 
