//
// Created by demphi on 2022/3/23.
//

#ifndef _GAITSCHEDULER_
#define _GAITSCHEDULER_


struct GaitData{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GaitData(){zero();}
    void zero(){
        // Stop any gait transitions
        next_gait_ = cur_gait_;

        // General Gait descriptors
        period_time_nominal_ = 0.0;      // overall period time to scale
        initial_phase_ = 0.0;           // initial phase to offset
        switching_phase_nominal_ = 0.0;  // nominal phase to switch contacts

        // Enable flag for each foot
        gait_enabled_ = Vec4::Zero();  // enable gait controlled legs

        // Time based descriptors
        period_time_ = Vec4::Zero();           // overall gait period time
        time_stance_ = Vec4::Zero();           // total stance time
        time_swing_ = Vec4::Zero();            // total swing time
        time_stance_remaining_ = Vec4::Zero();  // stance time remaining
        time_swing_remaining_ = Vec4::Zero();   // swing time remaining

        // Phase based descriptors
        switching_phase_ = Vec4::Zero();  // phase to switch to swing
        phase_variable_ = Vec4::Zero();   // overall gait phase for each foot
        phase_offset_ = Vec4::Zero();     // nominal gait phase offsets
        phase_scale_ = Vec4::Zero();      // phase scale relative to variable
        phase_stance_ = Vec4::Zero();     // stance subphase
        phase_swing_ = Vec4::Zero();      // swing subphase

        // Contact descriptors
        contact_state_schedule_ = Eigen::Vector4i::Zero();
        contact_state_prev_ = Eigen::Vector4i::Zero();
        touch_down_schedule_ = Eigen::Vector4i::Zero();
        lift_off_schedule_ = Eigen::Vector4i::Zero();

    }
    // the current gait type
    quad::STATE_TYPE cur_gait_;
    quad::STATE_TYPE next_gait_;

    std::string gait_name_;

    // Gait descriptors
    double period_time_nominal_;      // overall period time to scale
    double initial_phase_;           // initial phase to offset
    double switching_phase_nominal_;  // nominal phase to switch contacts

    // Enable gait control for each leg
    Vec4 gait_enabled_;

    // Time based descriptors
    Vec4 period_time_;           // overall foot scaled gait period time
    Vec4 time_stance_;           // total stance time
    Vec4 time_swing_;            // total swing time
    Vec4 time_stance_remaining_;  // stance time remaining
    Vec4 time_swing_remaining_;   // swing time remaining

    // Phase based descriptors
    Vec4 switching_phase_;  // phase to switch to swing
    Vec4 phase_variable_;   // overall gait phase for each foot
    Vec4 phase_offset_;     // nominal gait phase offsets
    Vec4 phase_scale_;      // phase scale relative to variable
    Vec4 phase_stance_;     // stance subphase
    Vec4 phase_swing_;      // swing subphase

    // Contact descriptors
    Eigen::Vector4i contact_state_schedule_;
    Eigen::Vector4i contact_state_prev_;
    Eigen::Vector4i touch_down_schedule_;
    Eigen::Vector4i lift_off_schedule_;

};

class GaitScheduler{
public:
    double dt_, dphase_;

    // Parameters for modify gait
    double period_time_natural_, switching_phase_natural_;
    GaitData gait_data_;

    GaitScheduler(quad::STATE_TYPE _gait_type, double _dt){
        ROS_INFO("[GAIT] Initalize Gait Scheduler!");
        this->gait_data_.cur_gait_ = _gait_type;
        this->gait_data_.zero();
        CreateGait();
        this->period_time_natural_ = this->gait_data_.period_time_nominal_;
        this->switching_phase_natural_ = this->gait_data_.switching_phase_nominal_;
        this->dt_ = _dt;
    }
    ~GaitScheduler(){

    }

    /*!
    * Executes the Gait Schedule step to calculate values for the defining gait parameters.
    */
    void step(){
        for (int foot = 0; foot < 4; foot++) {
            this->gait_data_.contact_state_prev_(foot) = this->gait_data_.contact_state_schedule_(foot);
            if(this->gait_data_.gait_enabled_(foot) == 1){
                // Get the dphase for iterate
                this->dphase_ = this->gait_data_.phase_scale_(foot) *
                        (this->dt_ / this->gait_data_.period_time_nominal_);
                // Update each foot's current phase
                this->gait_data_.phase_variable_(foot) =
                        fmod((this->gait_data_.phase_variable_(foot)+dphase_), 1);
                // In stance phase
                if(this->gait_data_.phase_variable_(foot) < this->gait_data_.switching_phase_(foot)){
                    this->gait_data_.contact_state_schedule_(foot) = 1;
                    this->gait_data_.phase_stance_(foot) =
                            this->gait_data_.phase_variable_(foot) / this->gait_data_.switching_phase_(foot);
                    // Foot is in stance, no swing time remaining
                    this->gait_data_.time_swing_remaining_(foot) = -1.0;
                    //Calculate the remaining time in stance
                    this->gait_data_.time_stance_remaining_(foot) =
                            this->gait_data_.period_time_(foot) *
                                (this->gait_data_.switching_phase_(foot)-this->gait_data_.phase_variable_(foot));
                    // First contact signifies scheduled touchdown
                    if(this->gait_data_.contact_state_prev_(foot) == 0){
                        //set touch down flag to 1
                        this->gait_data_.touch_down_schedule_(foot) = 1;
                    }else{
                        this->gait_data_.touch_down_schedule_(foot) = 0;
                    }

                }
                // In swing phase
                else{
                    this->gait_data_.contact_state_schedule_(foot) = 0;
                    this->gait_data_.phase_stance_(foot) = 1.0;
                    // cur swing phase in the whole swing phase(=1)
                    this->gait_data_.phase_swing_(foot) =
                            (this->gait_data_.phase_variable_(foot)-this->gait_data_.switching_phase_(foot)) /
                                    (1.0 - this->gait_data_.switching_phase_(foot));
                    // Foot is in swing, no stance time remaining
                    this->gait_data_.time_stance_remaining_(foot) = -1.0;
                    // Calculate the remaining time in swing
                    this->gait_data_.time_swing_remaining_(foot) =
                            this->gait_data_.period_time_(foot) * (1.0 - this->gait_data_.phase_variable_(foot));
                    // First contact signifies scheduled touchdown
                    if(this->gait_data_.contact_state_prev_(foot) == 1){
                        this->gait_data_.lift_off_schedule_(foot) = 1;
                    }else{
                        this->gait_data_.lift_off_schedule_(foot) = 0;
                    }
                }
            }
            else{
                //Leg is not enabled
                this->gait_data_.phase_variable_(foot) = 0.0;
                this->gait_data_.contact_state_schedule_(foot) = 0;

            }
        }
    }

    /*!
     *  Loading the gait parameters for different gait type
     */
    void CreateGait(){
        ROS_INFO("[GAIT] Loading parameters");
        switch (this->gait_data_.cur_gait_) {
            case quad::STATE_TYPE::WALK:
                this->gait_data_.gait_name_ = "WALK";
                this->gait_data_.gait_enabled_ << 1, 1, 1, 1;
                this->gait_data_.period_time_nominal_ = 1.25;
                this->gait_data_.initial_phase_ = 0.0;
                this->gait_data_.switching_phase_nominal_ = 0.8;
                this->gait_data_.phase_offset_ << 0, 0.5, 0.25, 0.75;
                this->gait_data_.phase_scale_ << 1.0, 1.0, 1.0, 1.0;
                break;
            case quad::STATE_TYPE::TROT:
                this->gait_data_.gait_name_ = "TROT";
                this->gait_data_.gait_enabled_ << 1, 1, 1, 1;
                this->gait_data_.period_time_nominal_ = 0.5;
                //解决初始迭代问题
                this->gait_data_.initial_phase_ = -0.02;
                this->gait_data_.switching_phase_nominal_ = 0.5;
                this->gait_data_.phase_offset_ << 0, 0.5, 0.5, 0;
                this->gait_data_.phase_scale_ << 1.0, 1.0, 1.0, 1.0;
                break;
        }
        CalculateAuxiliaryGaitData();
    }

    /*!
     * Set the gait parameters for each foot
     */
    void CalculateAuxiliaryGaitData(){
        for (int foot = 0; foot < 4; foot++) {
            if(this->gait_data_.gait_enabled_(foot)==1){
                // The scaled period time for each foot
                this->gait_data_.period_time_(foot) =
                        this->gait_data_.period_time_nominal_ / this->gait_data_.phase_scale_(foot);

                // Phase at which to switch the foot from stance to swing
                this->gait_data_.switching_phase_(foot) = this->gait_data_.switching_phase_nominal_;

                // Initialize the phase variables according to offset
                this->gait_data_.phase_variable_(foot) =
                        this->gait_data_.initial_phase_ + this->gait_data_.phase_offset_(foot);

                // Find the total stance time over the gait cycle
                this->gait_data_.time_stance_(foot) =
                        this->gait_data_.period_time_(foot) * this->gait_data_.switching_phase_(foot);

                // Find the total swing time over the gait cycle
                this->gait_data_.time_swing_(foot) =
                        this->gait_data_.period_time_(foot) * (1.0-this->gait_data_.switching_phase_(foot));
            }
            else{
                this->gait_data_.period_time_(foot) = 0.0;
                this->gait_data_.switching_phase_(foot) = 0.0;
                this->gait_data_.phase_variable_(foot) = 0.0;
                this->gait_data_.time_stance_(foot) = 0.0;
                this->gait_data_.time_swing_(foot) = 1.0;
            }
        }
    }
};

#endif 
