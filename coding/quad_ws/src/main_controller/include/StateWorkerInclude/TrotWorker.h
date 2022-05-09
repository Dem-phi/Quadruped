//
// Created by demphi on 2021/10/1.
//

#ifndef _TROTWORKER_
#define _TROTWORKER_

class TrotWorker:public StateWorker{
private:
    const double foot2ground_ = 0.1;
    int count_;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_cur;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_cur;
    Eigen::Matrix<float, 1, NUM_LEG> spline_time;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_target;
    Eigen::Matrix<double, 3, NUM_LEG> foot_pos_error;
    Eigen::Matrix<double, 3, NUM_LEG> foot_vel_error;

public:
    ros::NodeHandle nh_;

    GaitScheduler* model_GS = new GaitScheduler(quad::TROT, 0.01);
    FootSwingTrajectory* model_FST[4];
    BezierUtils bezierUtils[NUM_LEG];

    virtual void run(STATE_INTERIOR *cur_state);
    virtual bool is_finished();
    TrotWorker(ros::NodeHandle &nh);
    ~TrotWorker();
};

TrotWorker::TrotWorker(ros::NodeHandle &nh) {
    this->nh_ = nh;
    this->foot_pos_cur.setZero();
    this->foot_vel_cur.setZero();
    this->spline_time.setZero();
    this->foot_pos_target.setZero();
    this->foot_vel_target.setZero();
    this->foot_pos_error.setZero();
    this->foot_vel_error.setZero();

    for (int i = 0; i < 4; i++) {
        this->model_FST[i] = new FootSwingTrajectory();
    }
}

TrotWorker::~TrotWorker() {
    delete this->model_GS;
    for (int i = 0; i < 4; i++) {
        delete this->model_FST[i];
    }
}

void TrotWorker::run(STATE_INTERIOR *cur_state) {
    if(this->count_++ >= 100){
        ROS_INFO("Trotting");
        this->count_ = 0;
    }

    /*! Update state interior data */
    //this->model_GS->step();
    //cur_state->phase_variable = this->model_GS->gait_data_.phase_variable_;

    /*! Get foot hold position*/
    cur_state->foot_pDes_robot = cur_state->foot_p_bias;
    for (int foot = 0; foot < NUM_LEG; foot++){
        /*! Update Contact info */
        //cur_state->contacts[foot] = this->model_GS->gait_data_.contact_state_schedule_(foot);
        cur_state->gait_counter(foot) = cur_state->gait_counter(foot) + cur_state->gait_counter_speed(foot);
        cur_state->gait_counter(foot) = std::fmod(cur_state->gait_counter(foot), cur_state->counter_per_gait);
        if(cur_state->gait_counter(foot) <= cur_state->counter_per_swing){
            cur_state->contacts[foot] = 1;
        }else{
            cur_state->contacts[foot] = 0;
        }

/*        double delta_x = sqrt(abs(cur_state->foot_p_bias.block<3, 1>(0, foot).z())/9.8)*(cur_state->cur_vel_b.x()-cur_state->command_vel.x())
            +((this->model_GS->gait_data_.switching_phase_nominal_/this->model_GS->dphase_)*cur_state->plan_dt)/2*cur_state->command_vel.x();
        double delta_y = sqrt(abs(cur_state->foot_p_bias.block<3, 1>(0, foot).z())/9.8)*(cur_state->cur_vel_b.y()-cur_state->command_vel.y())
            +((this->model_GS->gait_data_.switching_phase_nominal_/this->model_GS->dphase_)*cur_state->plan_dt)/2*cur_state->command_vel.y();*/
        double delta_x = sqrt(abs(cur_state->foot_p_bias(2))/9.8)*(cur_state->cur_vel_b.x()-cur_state->command_vel.x())
                         +0.2/2.0*cur_state->command_vel.x();
        double delta_y = sqrt(abs(cur_state->foot_p_bias(2))/9.8)*(cur_state->cur_vel_b.y()-cur_state->command_vel.y())
                         +0.2/2.0*cur_state->command_vel.y();

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


    /*! Get Swing foot Target */
    for (int foot = 0; foot < NUM_LEG; foot++) {
        this->foot_pos_cur.block<3, 1>(0, foot) = cur_state->rotate_matrix_z.transpose() * cur_state->foot_p_abs.block<3, 1>(0, foot);
        //this->foot_pos_cur.block<3, 1>(0, foot) = cur_state->foot_p_robot.block<3, 1>(0, foot);
        if(/*this->model_GS->gait_data_.time_stance_remaining_(foot)>=0*/
        cur_state->gait_counter(foot)<= cur_state->counter_per_swing){
            // keep stance
            spline_time(foot) = 0.0;
            cur_state->foot_pos_start.block<3, 1>(0, foot) = foot_pos_cur.block<3, 1>(0, foot);
        }
        else{
/*            spline_time(foot) = float(this->model_GS->gait_data_.phase_variable_(foot)-this->model_GS->gait_data_.switching_phase_(foot))
                    / float(this->model_GS->gait_data_.switching_phase_(foot));*/
            //spline_time(foot) = float(1.0) - float(this->model_GS->gait_data_.time_swing_remaining_(foot));
            spline_time(foot) = float (cur_state->gait_counter(foot)-cur_state->counter_per_swing)
                                / float(cur_state->counter_per_swing);
        }

        /*! Bezier Curve */
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
        cur_state->foot_forces_swing.block<3, 1>(0, foot) = foot_pos_error.block<3, 1>(0, foot).cwiseProduct(cur_state->kp_foot.block<3, 1>(0, foot))
                                    + foot_vel_error.block<3, 1>(0, foot).cwiseProduct(cur_state->kd_foot.block<3, 1>(0, foot));
    }

    /*! older version */
    /*for (int foot = 0; foot < 4; foot++) {

        this->model_FST[foot]->setHeight(this->foot2ground_);
        *//*! at the beginning of stance *//*
        if(this->model_GS->gait_data_.phase_variable_(foot) <= 0.01){
            // Get the Bezier curve
            this->model_FST[foot]->setInitialPosition(cur_state->foot_p_robot.block<3,1>(0, foot));
            Vec3 final_position_ = cur_state->foot_p_bias.block<3, 1>(0, foot)
                                 - cur_state->command_vel*this->model_GS->period_time_natural_;
            this->model_FST[foot]->setFinalPosition(final_position_);
            this->model_FST[foot]->computeStanceTrajectoryBezier(this->model_GS->gait_data_.phase_stance_(foot),
                                                           this->model_GS->gait_data_.time_stance_(foot));
        }*//*! at the beginning of swing *//*
        else if(this->model_GS->gait_data_.phase_variable_(foot) <= 0.51 &&
                this->model_GS->gait_data_.phase_variable_(foot) >= 0.49){
            this->model_FST[foot]->setInitialPosition(cur_state->foot_p_robot.block<3,1>(0, foot));
            Vec3 final_position_ = cur_state->foot_p_bias.block<3, 1>(0, foot)
                                 + cur_state->command_vel*this->model_GS->period_time_natural_;
            this->model_FST[foot]->setFinalPosition(final_position_);
            this->model_FST[foot]->computeSwingTrajectoryBezier(this->model_GS->gait_data_.phase_swing_(foot),
                                                                this->model_GS->gait_data_.time_swing_(foot));
        }*//*! during the stance *//*
        else if(this->model_GS->gait_data_.time_stance_remaining_(foot)>=0){
            this->model_FST[foot]->computeStanceTrajectoryBezier(this->model_GS->gait_data_.phase_stance_(foot),
                                                           this->model_GS->gait_data_.time_stance_(foot));
        }
        *//*! during the swing*//*
        else if(this->model_GS->gait_data_.time_swing_remaining_(foot)>=0){
            this->model_FST[foot]->computeSwingTrajectoryBezier(this->model_GS->gait_data_.phase_swing_(foot),
                                                           this->model_GS->gait_data_.time_swing_(foot));
        }
        cur_state->foot_pDes.block<3, 1>(0, foot) = this->model_FST[foot]->getPosition();
        cur_state->foot_vDes.block<3, 1>(0, foot) = this->model_FST[foot]->getVelocity();
        cur_state->contacts[foot] = this->model_GS->gait_data_.contact_state_schedule_(foot);

    }*/

}

bool TrotWorker::is_finished() {
    return false;
}

#endif 
