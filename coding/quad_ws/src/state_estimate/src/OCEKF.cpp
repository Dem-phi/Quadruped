//
// Created by demphi on 2021/10/26.
//
#include "OCEKF.h"

namespace LSE{
    OCEKF::OCEKF(Controller* pController){
        this->pController_ = pController;
    }

    OCEKF::~OCEKF(){

    }

    void OCEKF::predict(){

    }

    void OCEKF::update(const double& t){
        double tNew_safe = t;
        tNew_safe = std::min(tNew_safe, this->pController_->Imu_Meas_List.rbegin()->first+this->pController_->tImu_);
    }

    void OCEKF::loadParam(){

    }

    void OCEKF::filterState(OCEKF::InternState &x, const double &t) {

    }

    void OCEKF::predictState(OCEKF::InternState &x, const double &t, const _imu_meas_ &imu_meas) {

    }

    void OCEKF::updateState(OCEKF::InternState &x, const _encoder_meas_ &encoder_meas) {

    }


}