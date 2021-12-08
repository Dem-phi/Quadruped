//
// Created by demphi on 2021/10/1.
//

#ifndef _TROTWORKER_
#define _TROTWORKER_

class TrotWorker:public StateWorker{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_angle;
    std_msgs::Float64MultiArray msg_angle;

    CAR* model_CAR = new CAR(1.0, 0.4, 0.02, quad::TROT_BETA);
    Hopf* model_Hopf = new Hopf({8.3, 5.3}, quad::TROT_BETA, quad::TROT_PHI);
    Eigen::Vector2f amplitude;

    virtual void run();
    virtual bool is_finished();
    TrotWorker(ros::NodeHandle &nh);
    ~TrotWorker();
};

TrotWorker::TrotWorker(ros::NodeHandle &nh) {
    this->nh = nh;
    this->pub_angle = this->nh.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle", 1);
    this->amplitude = model_CAR->Calculate_amplitude();
    //init
    this->msg_angle.data.resize(12);
}

TrotWorker::~TrotWorker() {
    delete this->model_CAR;
}

void TrotWorker::run() {
    ROS_INFO("Trotting");
    std::cout << amplitude.x() << "   " << amplitude.y() << std::endl;
}

bool TrotWorker::is_finished() {
    return false;
}

#endif 
