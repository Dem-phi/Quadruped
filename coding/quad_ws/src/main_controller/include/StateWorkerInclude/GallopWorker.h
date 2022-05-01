//
// Created by demphi on 2021/10/1.
//

#ifndef _GALLOPWORKER_
#define _GALLOPWORKER_

class GallopWorker:public StateWorker{
private:
    ros::Publisher angle_gazebo_pub_, angle_real_pub_;

public:
    ros::NodeHandle nh_;

    std_msgs::Float64MultiArray angle_gazebo_data_;

    std_msgs::Float64MultiArray angle_real_data_;

    CAR* model_CAR = new CAR(1.6, 0.4, 0.04, quad::GALLOP_BETA);
    Hopf* model_Hopf = new Hopf(model_CAR->Calculate_amplitude(), quad::GALLOP_BETA, quad::GALLOP_PHI);
    Eigen::Vector2f amplitude;

    virtual void run(STATE_INTERIOR *cur_state);
    virtual bool is_finished();

    GallopWorker(ros::NodeHandle &nh);
    ~GallopWorker();

};

GallopWorker::GallopWorker(ros::NodeHandle &nh) {
    this->nh_ = nh;
    this->angle_gazebo_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle_gazebo", 1);
    this->angle_real_pub_ = this->nh_.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle", 1);
    this->amplitude = model_CAR->Calculate_amplitude();

    /*! Init some param*/
    this->angle_gazebo_data_.data.resize(12);
    this->angle_real_data_.data.resize(3);

}

GallopWorker::~GallopWorker() {
    delete this->model_CAR;
    delete this->model_Hopf;

}

void GallopWorker::run(STATE_INTERIOR *cur_state) {
    ROS_INFO("Galloping");
    std::cout << this->amplitude << std::endl;
    /*! LF should->hip->knee (0, 1, 2)*/
    this->angle_gazebo_data_ = model_Hopf->CalculateAngle(this->angle_gazebo_data_);

    /*! add some bias, remap to the gazebo, */
    for (int i = 0; i < 4; i++) {
        this->angle_gazebo_data_.data[3*i] = 0.0;
        this->angle_gazebo_data_.data[3*i+1] += quad::GAZEBO_BIAS.y();
        this->angle_gazebo_data_.data[3*i+2] += quad::GAZEBO_BIAS.z();

    }
    this->angle_real_pub_.publish(this->angle_real_data_);
    /*! pub to gazebo */
    this->angle_gazebo_pub_.publish(this->angle_gazebo_data_);
}

bool GallopWorker::is_finished() {
    return false;
}

#endif
