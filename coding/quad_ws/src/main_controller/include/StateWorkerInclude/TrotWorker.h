//
// Created by demphi on 2021/10/1.
//

#ifndef _TROTWORKER_
#define _TROTWORKER_

class TrotWorker:public StateWorker{
private:
    const double foot2ground_ = 0.1;
    int count_;
    int mpc_delay;

public:
    ros::NodeHandle nh_;

    GaitScheduler* model_GS = new GaitScheduler(quad::TROT, 0.01);
    FootSwingTrajectory* model_FST[4];

    virtual void run(STATE_INTERIOR *cur_state);
    virtual bool is_finished();
    TrotWorker(ros::NodeHandle &nh);
    ~TrotWorker();
};

TrotWorker::TrotWorker(ros::NodeHandle &nh) {
    this->nh_ = nh;

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
    cur_state->gait_type = quad::TROT;
    /*! Update state interior data */
    this->model_GS->step();
    cur_state->phase_variable = this->model_GS->gait_data_.phase_variable_;
    for (int foot = 0; foot < 4; foot++) {
        this->model_FST[foot]->setHeight(this->foot2ground_);
        /*! at the beginning of stance */
        if(this->model_GS->gait_data_.phase_variable_(foot) <= 0.01){
            // Get the Bezier curve
            this->model_FST[foot]->setInitialPosition(cur_state->foot_p_robot.block<3,1>(0, foot));
            Vec3 final_position_ = cur_state->foot_p_bias.block<3, 1>(0, foot)
                                 - cur_state->command_vel*this->model_GS->period_time_natural_;
            this->model_FST[foot]->setFinalPosition(final_position_);
            this->model_FST[foot]->computeStanceTrajectoryBezier(this->model_GS->gait_data_.phase_stance_(foot),
                                                           this->model_GS->gait_data_.time_stance_(foot));
        }/*! at the beginning of swing */
        else if(this->model_GS->gait_data_.phase_variable_(foot) <= 0.51 &&
                this->model_GS->gait_data_.phase_variable_(foot) >= 0.49){
            this->model_FST[foot]->setInitialPosition(cur_state->foot_p_robot.block<3,1>(0, foot));
            Vec3 final_position_ = cur_state->foot_p_bias.block<3, 1>(0, foot)
                                 + cur_state->command_vel*this->model_GS->period_time_natural_;
            this->model_FST[foot]->setFinalPosition(final_position_);
            this->model_FST[foot]->computeSwingTrajectoryBezier(this->model_GS->gait_data_.phase_swing_(foot),
                                                                this->model_GS->gait_data_.time_swing_(foot));
        }/*! during the stance */
        else if(this->model_GS->gait_data_.time_stance_remaining_(foot)>=0){
            this->model_FST[foot]->computeStanceTrajectoryBezier(this->model_GS->gait_data_.phase_stance_(foot),
                                                           this->model_GS->gait_data_.time_stance_(foot));
        }
        /*! during the swing*/
        else if(this->model_GS->gait_data_.time_swing_remaining_(foot)>=0){
            this->model_FST[foot]->computeSwingTrajectoryBezier(this->model_GS->gait_data_.phase_swing_(foot),
                                                           this->model_GS->gait_data_.time_swing_(foot));
        }
        cur_state->foot_pDes.block<3, 1>(0, foot) = this->model_FST[foot]->getPosition();
        cur_state->foot_vDes.block<3, 1>(0, foot) = this->model_FST[foot]->getVelocity();
        cur_state->contacts[foot] = this->model_GS->gait_data_.contact_state_schedule_(foot);

    }
    //std::cout << cur_state->contacts[0]  << cur_state->contacts[1] << cur_state->contacts[2] << cur_state->contacts[3]<< std::endl;
}

bool TrotWorker::is_finished() {
    return false;
}

#endif 
