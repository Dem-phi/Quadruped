//
// Created by demphi on 2021/10/1.
//

#ifndef _GALLOPWORKER_
#define _GALLOPWORKER_

class GallopWorker:public StateWorker{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_angle;
    std_msgs::Float64MultiArray msg_angle;

    CAR* model_CAR = new CAR(1.6, 0.4, 0.04, quad::GALLOP_BETA);
    Hopf* model_Hopf = new Hopf({13.4, 10.0}, quad::GALLOP_BETA, quad::GALLOP_PHI);
    Eigen::Vector2f amplitude;

    virtual void run();
    virtual bool is_finished();

    GallopWorker(ros::NodeHandle &nh);
    ~GallopWorker();

};

GallopWorker::GallopWorker(ros::NodeHandle &nh) {
    this->nh = nh;
    this->pub_angle = this->nh.advertise<std_msgs::Float64MultiArray>(
            "/quad/set_angle", 1);
    this->amplitude = model_CAR->Calculate_amplitude();
    //init
    this->msg_angle.data.resize(12);
}

GallopWorker::~GallopWorker() {
    delete this->model_CAR;
    delete this->model_Hopf;

}

void GallopWorker::run() {
    ROS_INFO("Galloping");
    //cout << amplitude.x() << "   " << amplitude.y() << endl;
    this->msg_angle = model_Hopf->CalculateAngle(this->msg_angle);
    pub_angle.publish(msg_angle);
}

bool GallopWorker::is_finished() {
    return false;
}

#endif
