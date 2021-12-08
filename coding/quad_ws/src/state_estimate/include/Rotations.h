/*!
 *
 */

#ifndef _ROTATIONS_
#define _ROTATIONS_

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>

namespace LSE{
namespace Rotations{

/*! Converts vector to sqew matrix
 * @return	corresponding sqew-matrix
 * @param[in] 	v	vector
 */
inline Eigen::Matrix3d VecToSqew(const Eigen::Vector3d& v){
    Eigen::Matrix3d M;
    M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
    return M;
}

/*! Converts a quaternion to a rotation matrix
 * @return 	corresponding rotation matrix
 * @param[in]	q	quaternion
 */
inline Eigen::Matrix3d QuatToRotMat(const Eigen::Quaternion<double>& q){
    Eigen::Matrix3d M;
    M = q.toRotationMatrix();
    return M;
}

/*! Converts a quaternion to a rotation vector
 * @return 	corresponding rotation vector
 * @param[in]	q	quaternion
 */
inline Eigen::AngleAxisd quatToRotVec(const Eigen::Quaternion<double>& q){
    Eigen::AngleAxisd v;
    v = q;
    return v;
}



}
}


#endif 
