//
// Created by demphi on 2021/10/1.
//

#include <FiniteStateMachine.h>
#include "time.h"
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;

    FSM Finite_State_Machine(nh);
    sleep(2);
    Finite_State_Machine.build_ScheduleTable(
            quad::STAND,
            //quad::WALK,
            quad::TROT,
//           quad::PACE,
//           quad::GALLOP,
            quad::END
            );

    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);


    std::thread Main_update_thread([&]{
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  // bool res = app.exec();
        ros::Duration dt(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            ros::Duration(0.0025).sleep();
            // get t and dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;

            auto t1 = std::chrono::high_resolution_clock::now();
            //Update main information
            Finite_State_Machine.loop(dt.toSec());
            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms_double = t2 - t1;
        }
    });

    std::thread Update_contact_force_thread([&](){
        // prepare variables to monitor time and control the while loop
        ros::Time start = ros::Time::now();
        ros::Time prev = ros::Time::now();
        ros::Time now = ros::Time::now();  // bool res = app.exec();
        ros::Duration dt(0);

        while (control_execute.load(std::memory_order_acquire) && ros::ok()) {
            ros::Duration(0.0025).sleep();

            // get t and dt
            now = ros::Time::now();
            dt = now - prev;
            prev = now;
            ros::Duration elapsed = now - start;

            auto t1 = std::chrono::high_resolution_clock::now();

            // compute desired ground forces
            Finite_State_Machine.Update_MPC(dt.toSec());

            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms_double = t2 - t1;
            std::cout << "MPC solution is updated in " << ms_double.count() << "ms" << std::endl;

        }

    });

    ros::AsyncSpinner spinner(12);
    spinner.start();

    Main_update_thread.join();
    Update_contact_force_thread.join();



    ros::waitForShutdown();
    return 0;
}