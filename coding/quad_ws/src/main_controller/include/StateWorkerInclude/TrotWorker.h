//
// Created by demphi on 2021/10/1.
//

#ifndef _TROTWORKER_
#define _TROTWORKER_

class TrotWorker:public StateWorker{
public:
    ros::NodeHandle nh;


    virtual void run();
    virtual bool is_finished();
    TrotWorker(ros::NodeHandle &nh);
    ~TrotWorker();
};

TrotWorker::TrotWorker(ros::NodeHandle &nh) {

}

TrotWorker::~TrotWorker() {

}

void TrotWorker::run() {

}

bool TrotWorker::is_finished() {
    return false;
}

#endif 
