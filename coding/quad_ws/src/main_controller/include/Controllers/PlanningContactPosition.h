//
// Created by demphi on 2022/3/30.
//

/*! no use */

#ifndef _PLANNINGCONTACTPOSITION_
#define _PLANNINGCONTACTPOSITION_

struct PlanningData{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PlanningData(){zero();}
    void zero(){
        planning_enabled_ = Vec4::Zero();
        init_position_ = Vec3::Zero();
        swing_contact_position_ = Vec3::Zero();
        stance_contact_position_ = Vec3::Zero();
        for (int i = 0; i < 4; i++) {
            target_position_[i] = Vec3::Zero();
        }
    }
    //the current gait type
    quad::STATE_TYPE cur_gait_;

    double step_size_;

    // Enable modify contact position
    Vec4 planning_enabled_;

    // init stand position
    Vec3 init_position_;

    // target position descriptors
    Vec3 swing_contact_position_;
    Vec3 stance_contact_position_;

    // final target sent to FootSwingTrajectory
    Vec3 target_position_[4];

};

class PlanningContactPosition{
public:
    PlanningData planning_data_;

    PlanningContactPosition(quad::STATE_TYPE _gait_type, double _step_size){
        this->planning_data_.cur_gait_ = _gait_type;
        this->planning_data_.zero();
        this->planning_data_.step_size_ = _step_size;
    }

    ~PlanningContactPosition(){
    }

    void step(int _leg, double _flag){
        if(this->planning_data_.planning_enabled_(_leg) == 1){
            //use vision for planning
        }else{
            if(_flag <= 0.01){
                this->planning_data_.target_position_[_leg] = this->planning_data_.stance_contact_position_;
            }else{
                this->planning_data_.target_position_[_leg] = this->planning_data_.swing_contact_position_;
            }
        }
        if(_leg == 0|| _leg == 1){
            this->planning_data_.target_position_[_leg].x() += quad::X_OFFSET;
        }else{
            this->planning_data_.target_position_[_leg].x() -= quad::X_OFFSET;
        }
        if(_leg == 0|| _leg == 3){
            this->planning_data_.target_position_[_leg].y() += quad::Y_OFFSET;
        }else{
            this->planning_data_.target_position_[_leg].y() -= quad::Y_OFFSET;
        }
/*        std::cout << this->planning_data_.target_position_[_leg] <<std::endl;*/
    }

    /*!
     *  Update parameters for planning
     */
    void Planning(Vec3 _init_position){
        this->planning_data_.init_position_.x() = _init_position.x()-quad::X_OFFSET;
        this->planning_data_.init_position_.y() = _init_position.y()-quad::Y_OFFSET;
        this->planning_data_.init_position_.z() = _init_position.z();
        this->planning_data_.planning_enabled_ << 0.0, 0.0, 0.0, 0.0;
        ROS_INFO("[PLANNING] Loading parameters");
        switch (this->planning_data_.cur_gait_){
            case quad::STATE_TYPE::WALK:
                this->planning_data_.swing_contact_position_ << this->planning_data_.step_size_/2, 0.0, this->planning_data_.init_position_.z();
                this->planning_data_.stance_contact_position_ << -this->planning_data_.step_size_/2, 0.0, this->planning_data_.init_position_.z();
                break;
            case quad::STATE_TYPE::TROT:
                this->planning_data_.swing_contact_position_ << this->planning_data_.step_size_/2, 0.0, this->planning_data_.init_position_.z();
                this->planning_data_.stance_contact_position_ << -this->planning_data_.step_size_/2, 0.0, this->planning_data_.init_position_.z();
                break;
        }
    }

};


#endif 
