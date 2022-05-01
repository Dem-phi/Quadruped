//
// Created by demphi on 2022/4/26.
//

#ifndef MAIN_CONTROLLER_UTILS_H
#define MAIN_CONTROLLER_UTILS_H

class Utils{
public:
    static Eigen::Matrix3d skew(Eigen::Vector3d vec){
        Eigen::Matrix3d temp;
        temp.setZero();
        temp <<        0, -vec.z(),  vec.y(),
                 vec.z(),        0, -vec.x(),
                -vec.y(),  vec.x(),         0;
        return temp;
    }
};

#endif 
