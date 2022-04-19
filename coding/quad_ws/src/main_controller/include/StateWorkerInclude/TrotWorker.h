//
// Created by demphi on 2021/10/1.
//

#ifndef _TROTWORKER_
#define _TROTWORKER_

class TrotWorker:public StateWorker{
private:
    const double velocity_ = 0.5;
    const double foot2ground_ = 0.1;
    // sign for update data firstly
    int flag = 1;
    // sign for footSwingTrajectory firstly
    Eigen::Vector4i flag_tra;

    Vec3 temp_start_position_[4], temp_end_position[4];

    ros::Publisher angle_gazebo_pub_, angle_real_pub_;
    ros::Publisher EKF_pub_;

public:
    ros::NodeHandle nh_;

    std_msgs::Float64MultiArray angle_gazebo_data_;
    std_msgs::Float64MultiArray angle_real_data_;
    std_msgs::Float64MultiArray EKF_data_;

    GaitScheduler* model_GS = new GaitScheduler(quad::TROT, 0.01);
    FootSwingTrajectory* model_FST[4];
    LegController* model_LegCtl = new LegController(this->velocity_, this->model_GS->period_time_natural_, 0.01);
    PlanningContactPosition * model_PCP = new PlanningContactPosition(quad::TROT, this->model_LegCtl->step_size_);
    StateEstimate* model_StateEstimate = new StateEstimate();

    virtual void run(quad::STATE_INFO cur_state);
    virtual bool is_finished();
    TrotWorker(ros::NodeHandle &nh);
    ~TrotWorker();
};

TrotWorker::TrotWorker(ros::NodeHandle &nh) {
    this->nh_ = nh;
    this->angle_gazebo_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle_gazebo", 1);
    this->angle_real_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle", 1);
    this->EKF_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/EKF", 1);

    this->flag_tra << 1,1,1,1;

    /*! Init some param*/
    this->angle_gazebo_data_.data.resize(12);
    this->angle_real_data_.data.resize(3);
    this->EKF_data_.data.resize(9); //position + velocity in world frame + velocity in body frame
    for (int i = 0; i < 4; i++) {
        this->model_FST[i] = new FootSwingTrajectory();
    }

}

TrotWorker::~TrotWorker() {
    delete this->model_GS;
    for (int i = 0; i < 4; i++) {
        delete this->model_FST[i];
    }
    delete this->model_LegCtl;
    delete this->model_PCP;
    delete this->model_StateEstimate;
}

void TrotWorker::run(quad::STATE_INFO cur_state) {
    if(this->flag == 1){
        ROS_INFO("Trotting");
        this->model_LegCtl->FirstUpdateData();
        this->flag = 0;
        this->model_PCP->Planning(this->model_LegCtl->model_kinematic->cur_p_[0]);
    }
    this->model_StateEstimate->UpdateOrientationData(cur_state);
    this->model_GS->step();
    for (int foot = 0; foot < 4; foot++) {
        this->model_FST[foot]->setHeight(this->foot2ground_);
        /*! at the beginning of stance */
        if(this->model_GS->gait_data_.phase_variable_(foot) <= 0.01){
            // Get the Bezier curve
            this->model_FST[foot]->setInitialPosition(this->model_LegCtl->leg_data_[foot].p);
            // Without planning contact position
            if(this->flag_tra(foot) == 1){
                Vec3 final_position_ = this->model_LegCtl->leg_data_[foot].p;
                final_position_.x() -= this->model_LegCtl->step_size_/2;
                this->model_FST[foot]->setFinalPosition(final_position_);
                this->flag_tra(foot) = 0;
            }else{
                Vec3 final_position_ = this->model_LegCtl->leg_data_[foot].p;
                final_position_.x() -= this->model_LegCtl->step_size_;
                this->model_FST[foot]->setFinalPosition(final_position_);
            }
            //this->model_PCP->step(foot, this->model_GS->gait_data_.phase_variable_(foot));
            //this->model_FST[foot]->setFinalPosition(this->model_PCP->planning_data_.target_position_[foot]);

            this->model_FST[foot]->computeStanceTrajectoryBezier(this->model_GS->gait_data_.phase_stance_(foot),
                                                           this->model_GS->gait_data_.time_stance_(foot));
        }/*! at the beginning of swing */
        else if(this->model_GS->gait_data_.phase_variable_(foot) <= 0.51 &&
                this->model_GS->gait_data_.phase_variable_(foot) >= 0.49){
            this->model_FST[foot]->setInitialPosition(this->model_LegCtl->leg_data_[foot].p);
            // Without planning contact position
            if(this->flag_tra(foot) == 1){
                Vec3 final_position_ = this->model_LegCtl->leg_data_[foot].p;
                final_position_.x() += this->model_LegCtl->step_size_/2;
                this->model_FST[foot]->setFinalPosition(final_position_);
                this->flag_tra(foot) = 0;
            }else{
                Vec3 final_position_ = this->model_LegCtl->leg_data_[foot].p;
                final_position_.x() += this->model_LegCtl->step_size_;
                this->model_FST[foot]->setFinalPosition(final_position_);
            }
            //this->model_PCP->step(foot, this->model_GS->gait_data_.phase_variable_(foot));
            //this->model_FST[foot]->setFinalPosition(this->model_PCP->planning_data_.target_position_[foot]);
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
        this->model_LegCtl->leg_command_[foot].pDes = this->model_FST[foot]->getPosition();
        this->model_LegCtl->leg_command_[foot].vDes = this->model_FST[foot]->getVelocity();
    }
    //std::cout << this->model_LegCtl->leg_data_[0].q*quad::Rad2Deg<< std::endl;
/*
    std::cout << this->model_LegCtl->leg_command_[3].pDes.x() << "      "
    << this->model_LegCtl->leg_command_[3].pDes.z() << "       " <<this->model_GS->gait_data_.phase_variable_(0)<< std::endl;
    std::cout << this->model_LegCtl->leg_command_[3].qDes.y()*quad::Rad2Deg << "       " <<
    this->model_LegCtl->leg_command_[3].qDes.z()*quad::Rad2Deg << "       " <<this->model_GS->gait_data_.phase_variable_(0) << std::endl;
*/
    /*! run linear kalman filter */
    this->model_StateEstimate->LinearKF(this->model_LegCtl->leg_data_, this->model_GS->gait_data_.phase_variable_);
    this->EKF_data_ = this->model_StateEstimate->getData();
    this->EKF_pub_.publish(this->EKF_data_);

    this->model_LegCtl->UpdateCommand();

    /*! map command to gazebo data msg*/
    this->model_LegCtl->SetData(&this->angle_gazebo_data_);

    this->angle_real_pub_.publish(this->angle_real_data_);
    /*! pub to gazebo */
    this->angle_gazebo_pub_.publish(this->angle_gazebo_data_);

    this->model_LegCtl->UpdateData();
}

bool TrotWorker::is_finished() {
    return false;
}

#endif 
