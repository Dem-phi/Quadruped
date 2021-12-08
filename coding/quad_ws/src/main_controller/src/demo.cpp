//
// Created by demphi on 2021/10/1.
//

#include <FiniteStateMachine.h>
#include "time.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    FSM Finite_State_Machine(nh);

    Finite_State_Machine.build_ScheduleTable(
            quad::STAND,
//            quad::TROT, 1.0
//            quad::PACE, 1.2,
//            quad::GALLOP, 1.6,
            quad::END
            );
    Finite_State_Machine.set_timer();


    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}