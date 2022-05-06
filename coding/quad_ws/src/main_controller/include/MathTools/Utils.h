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


class BezierUtils {
    // TODO: allow degree change? may not be necessary, we can stick to degree 4
public:
    BezierUtils () {
        curve_constructed = false;
        bezier_degree = 4;
    }
    // set of functions create bezier curves, get points, reset
    Eigen::Vector3d get_foot_pos_curve(float t,
                                       Eigen::Vector3d foot_pos_start,
                                       Eigen::Vector3d foot_pos_final,
                                       double terrain_pitch_angle){
        Eigen::Vector3d foot_pos_target;
        // X-axis
        std::vector<double> bezierX{foot_pos_start(0),
                                    foot_pos_start(0),
                                    foot_pos_final(0),
                                    foot_pos_final(0),
                                    foot_pos_final(0)};
        foot_pos_target(0) = bezier_curve(t, bezierX);

        // Y-axis
        std::vector<double> bezierY{foot_pos_start(1),
                                    foot_pos_start(1),
                                    foot_pos_final(1),
                                    foot_pos_final(1),
                                    foot_pos_final(1)};
        foot_pos_target(1) = bezier_curve(t, bezierY);

        // Z-axis
        std::vector<double> bezierZ{foot_pos_start(2),
                                    foot_pos_start(2),
                                    foot_pos_final(2),
                                    foot_pos_final(2),
                                    foot_pos_final(2)};
        bezierZ[1] += FOOT_SWING_CLEARANCE1;
        bezierZ[2] += FOOT_SWING_CLEARANCE2 + 0.5*sin(terrain_pitch_angle);
        foot_pos_target(2) = bezier_curve(t, bezierZ);

        return foot_pos_target;
    }

    bool reset_foot_pos_curve() {curve_constructed = false;}
private:
    double bezier_curve(double t, const std::vector<double> &P){
        std::vector<double> coefficients{1, 4, 6, 4, 1};
        double y = 0;
        for (int i = 0; i <= bezier_degree; i++) {
            y += coefficients[i] * std::pow(t, i) * std::pow(1 - t, bezier_degree - i) * P[i];
        }
        return y;
    }

    bool curve_constructed;
    float bezier_degree;
};


#endif 
