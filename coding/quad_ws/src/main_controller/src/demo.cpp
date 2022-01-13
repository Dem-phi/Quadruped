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
            //quad::WALK,
           quad::TROT,
//           quad::PACE,
//           quad::GALLOP,
            quad::END
            );
    Finite_State_Machine.set_timer();

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}