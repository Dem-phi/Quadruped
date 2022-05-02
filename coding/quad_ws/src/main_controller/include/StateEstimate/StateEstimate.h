//
// Created by demphi on 2022/4/6.
//

#ifndef MAIN_CONTROLLER_STATEESTIMATE_H
#define MAIN_CONTROLLER_STATEESTIMATE_H

#include "common_defination.h"
#include "common_include.h"
#include "Controllers/LegController.h"

struct StateEstimateData{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Quaterniond orientation_;
    Mat33 rBody_;
    Vec3 omega_body_;
    Vec3 omega_world_;
    Vec3 rpy_;
    Vec3 aBody_;
    Vec3 aWorld_;

    // output
    Vec3 position_;
    Vec3 vWorld_;
    Vec3 vBody_;
};

class StateEstimate{
public:
    const double imu_process_noise_position = 0.02;
    const double imu_process_noise_velocity = 0.02;
    const double foot_process_noise_position = 0.002;
    const double foot_sensor_noise_position = 0.01;
    const double foot_sensor_noise_velocity = 0.1;
    const double foot_height_sensor_noise = 0.001;

/*    const double imu_process_noise_position = 0;
    const double imu_process_noise_velocity = 0;
    const double foot_process_noise_position = 0;
    const double foot_sensor_noise_position = 0;
    const double foot_sensor_noise_velocity = 0;
    const double foot_height_sensor_noise = 0;*/

    StateEstimateData state_data_;

    StateEstimate(){
        InitParameters();
    }
    ~StateEstimate(){

    }

    void UpdateOrientationData(STATE_INTERIOR _state_interior){
        this->state_data_.orientation_ = _state_interior.quaternion;
        this->state_data_.rBody_ = _state_interior.rotate_matrix;

        this->state_data_.omega_body_ = _state_interior.b_angle_vel;

        this->state_data_.omega_world_ = this->state_data_.rBody_.transpose()*this->state_data_.omega_body_;

        this->state_data_.rpy_ = _state_interior.cur_rpy;
        this->state_data_.aBody_ = _state_interior.b_acc;
        this->state_data_.aWorld_ = this->state_data_.rBody_.transpose()*this->state_data_.aBody_;
    }

    void InitParameters(){
        double dt = 0.01;
        _xhat.setZero();
        _ps.setZero();
        _vs.setZero();
        _A.setZero();
        _A.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
        _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
        _A.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
        _A.block(6, 6, 12, 12) = Eigen::Matrix<double, 12, 12>::Identity();
        _B.setZero();
        _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
        C1 << Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero();
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
        C2 << Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity();
        _C.setZero();
        _C.block(0, 0, 3, 6) = C1;
        _C.block(3, 0, 3, 6) = C1;
        _C.block(6, 0, 3, 6) = C1;
        _C.block(9, 0, 3, 6) = C1;
        _C.block(0, 6, 12, 12) = -1 * Eigen::Matrix<double, 12, 12>::Identity();
        _C.block(12, 0, 3, 6) = C2;
        _C.block(15, 0, 3, 6) = C2;
        _C.block(18, 0, 3, 6) = C2;
        _C.block(21, 0, 3, 6) = C2;
        _C(27, 17) = 1;
        _C(26, 14) = 1;
        _C(25, 11) = 1;
        _C(24, 8) = 1;
        _P.setIdentity();
        _P = 100 * _P;
        _Q0.setIdentity();
        _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
        _Q0.block(3, 3, 3, 3) =
                (dt * 9.8f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
        _Q0.block(6, 6, 12, 12) = dt * Eigen::Matrix<double, 12, 12>::Identity();
        _R0.setIdentity();
    }

    /*!
     * Get the position and velocity
     * */
    void LinearKF(Eigen::Matrix<double, 3, quad::NUM_LEG> _foot_p, Eigen::Matrix<double, 3, quad::NUM_LEG> _foot_v, Vec4 _contact_phase){
        Eigen::Matrix<double, 18, 18> Q = Eigen::Matrix<double, 18, 18>::Identity();
        Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * this->imu_process_noise_position;
        Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * this->imu_process_noise_velocity;
        Q.block(6, 6, 12, 12) = _Q0.block(6, 6, 12, 12) * this->foot_process_noise_position;

        Eigen::Matrix<double, 28, 28> R = Eigen::Matrix<double, 28, 28>::Identity();
        R.block(0, 0, 12, 12) = _R0.block(0, 0, 12, 12) * this->foot_sensor_noise_position;
        R.block(12, 12, 12, 12) =
                _R0.block(12, 12, 12, 12) * this->foot_sensor_noise_velocity;
        R.block(24, 24, 4, 4) = _R0.block(24, 24, 4, 4) * this->foot_height_sensor_noise;

        int qindex = 0;
        int rindex1 = 0;
        int rindex2 = 0;
        int rindex3 = 0;

        /*!（2）定义三维矩阵 */
        Vec3 g(0, 0, -9.81);                                        // 1）定义重力矩阵
        Mat33 Rbod = this->state_data_.rBody_.transpose();   // 2）定义身体坐标系下的旋转矩阵，通过四元素转换而来
        Vec3 a = this->state_data_.aWorld_ + g;              // 3）定义世界坐标系下的加速度矩阵

        /*!（3）定义四维矩阵 */
        Vec4 pzs = Vec4::Zero();
        Vec4 trusts = Vec4::Zero();

        /*!（4）定义状态估计值矩阵：世界坐标下[p v p1 p2 p3 p4] */
        Vec3 p0, v0;
        p0 << _xhat[0], _xhat[1], _xhat[2];
        v0 << _xhat[3], _xhat[4], _xhat[5];

        /*!（5）卡尔曼滤波的状态预测部分,计算机器人躯干的位置、速度和触地状态 */
        for (int i = 0; i < 4; i++) {
            int i1 = 3 * i;
/*            Vec3 ph;                                        //计算相对于CoM的髋部位置，即机器人坐标系中腿的臀部位置
*/
            Vec3 p_rel = _foot_p.block<3, 1>(0, i);                  //计算世界坐标系下机器人真正的位置= 机器人坐标系中腿的臀部位置+腿的位置
            Vec3 dp_rel = _foot_v.block<3, 1>(0, i);                      //计算世界坐标系下机器人真正的速度= 状态估计器中每条的速度
            Vec3 p_f = Rbod * p_rel;                        //计算身体坐标系下机器人真正的位置= 身体坐标系下的旋转矩阵*世界坐标系下机器人真正的位置
            Vec3 dp_f =Rbod *(this->state_data_.omega_body_.cross(p_rel) + dp_rel);     //计算身体坐标系下机器人真正的速度=     身体坐标系下的旋转矩阵*（躯干坐标系中的角速度正交阵+世界坐标系下机器人真正的速度）


            qindex = 6 + i1;
            rindex1 = i1;
            rindex2 = 12 + i1;
            rindex3 = 24 + i;

            double trust = 1;                                     //定义触地标志位变量trust
            double phase = fmin(_contact_phase(i), 1);         //计算接触相序

            //T trust_window = T(0.25);
            double trust_window = 0.2;

            if (phase < trust_window) {
                trust = phase / trust_window;
            } else if (phase > (1 - trust_window)) {
                trust = (1 - phase) / trust_window;
            }

            double high_suspect_number(100);

            Q.block(qindex, qindex, 3, 3) =
                    (1 + (1 - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
            R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
            R.block(rindex2, rindex2, 3, 3) =
                    (1 + (1 - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
            R(rindex3, rindex3) =
                    (1 + (1 - trust) * high_suspect_number) * R(rindex3, rindex3);

            trusts(i) = trust;

            _ps.segment(i1, 3) = -p_f;
            _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
            pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));
        }

        /*!(6)卡尔曼滤波的状态融合部分(套公式)*/
        Eigen::Matrix<double, 28, 1> y;
        y << _ps, _vs, pzs;
        _xhat = _A * _xhat + _B * a;
        Eigen::Matrix<double, 18, 18> At = _A.transpose();
        Eigen::Matrix<double, 18, 18> Pm = _A * _P * At + Q;
        Eigen::Matrix<double, 18, 28> Ct = _C.transpose();
        Eigen::Matrix<double, 28, 1> yModel = _C * _xhat;
        Eigen::Matrix<double, 28, 1> ey = y - yModel;
        Eigen::Matrix<double, 28, 28> S = _C * Pm * Ct + R;

        // todo compute LU only once
        Eigen::Matrix<double, 28, 1> S_ey = S.lu().solve(ey);
        _xhat += Pm * Ct * S_ey;

        Eigen::Matrix<double, 28, 18> S_C = S.lu().solve(_C);
        _P = (Eigen::Matrix<double, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;

        Eigen::Matrix<double, 18, 18> Pt = _P.transpose();
        _P = (_P + Pt) / double(2);

        if (_P.block(0, 0, 2, 2).determinant() > double(0.000001))
        {
            _P.block(0, 2, 2, 16).setZero();
            _P.block(2, 0, 16, 2).setZero();
            _P.block(0, 0, 2, 2) /= double(10);
        }

        /*!(7)输出卡尔曼估计结果*/
        this->state_data_.position_ = _xhat.block(0, 0, 3, 1);     //1）输出卡尔曼滤波的躯干位置
        this->state_data_.vWorld_ = _xhat.block(3, 0, 3, 1);       //2）输出卡尔曼滤波的躯干速度，基于世界坐标系下的
        this->state_data_.vBody_ =                                 //3）输出卡尔曼滤波的躯干速度，基于身体坐标系下的
                this->state_data_.rBody_ * this->state_data_.vWorld_;

    }

    /*!
     * @return : the position of body
     * */
    Vec3 getPosition(){
        return this->state_data_.position_;
    }

    /*!
    * @return : the velocity in world frame
    * */
    Vec3 getVelocity(){
        return this->state_data_.vWorld_;
    }

    /*!
    * @return : the velocity in body frame
    * */
    Vec3 getVelocity_Body(){
        return this->state_data_.vBody_;
    }

    /*!
    * @return : the data of position velocity_w velocity body
    * */
    Eigen::Matrix<double, 3, 3> getData(){
        Eigen::Matrix<double, 3, 3> data_;
        data_.block<3, 1>(0, 0) = this->state_data_.position_;
        data_.block<3, 1>(0, 1) = this->state_data_.vWorld_;
        data_.block<3, 1>(0, 2) = this->state_data_.vBody_;
        return data_;
    }


private:
    Eigen::Matrix<double, 18, 1> _xhat; // x:=(r v p1 p2 p3 p4)
    Eigen::Matrix<double, 12, 1> _ps;   //
    Eigen::Matrix<double, 12, 1> _vs;   //
    Eigen::Matrix<double, 18, 18> _A;   // Fk
    Eigen::Matrix<double, 18, 18> _Q0;  //
    Eigen::Matrix<double, 18, 18> _P;   //
    Eigen::Matrix<double, 28, 28> _R0;  //
    Eigen::Matrix<double, 18, 3> _B;    //
    Eigen::Matrix<double, 28, 18> _C;   // Hk

};

#endif 
