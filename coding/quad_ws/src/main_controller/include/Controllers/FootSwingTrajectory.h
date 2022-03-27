/*!
 * @file FootSwingTrajectory.h
 * @brief Utility to generate foot swing trajectories.
 *
 * Currently uses Bezier curves like Cheetah 3 does
 */

#ifndef _FOOTSWINGTRAJECTORY_
#define _FOOTSWINGTRAJECTORY_

#include "MathTools/Interpolation.h"

/*!
 * A foot swing trajectory for a single foot
 */
class FootSwingTrajectory {
private:
    Vec3 _p0, _pf, _p, _v, _a;
    double _height;

public:
    /*!
     * Construct a new foot swing trajectory with everything set to zero
     */
    FootSwingTrajectory() {
        _p0.setZero();
        _pf.setZero();
        _p.setZero();
        _v.setZero();
        _a.setZero();
        _height = 0;
    }

    /*!
     * Set the starting location of the foot
     * @param p0 : the initial foot position
     */
    void setInitialPosition(Vec3 p0) {
        _p0 = p0;
    }

    /*!
     * Set the desired final position of the foot
     * @param pf : the final foot posiiton
     */
    void setFinalPosition(Vec3 pf) {
        _pf = pf;
    }

    /*!
     * Set the maximum height of the swing
     * @param h : the maximum height of the swing, achieved halfway through the swing
     */
    void setHeight(double h) {
        _height = h;
    }

    /*!
    * Compute foot swing trajectory with a bezier curve
    * @param phase : How far along we are in the swing (0 to 1)
    * @param swingTime : How long the swing should take (seconds)
    */
    void computeSwingTrajectoryBezier(double phase, double swingTime) {
        _p[0] = Interpolate::cubicBezier(_p0[0], _pf[0], phase);
        _v[0] = Interpolate::cubicBezierFirstDerivative(_p0[0], _pf[0], phase);
        _a[0] = Interpolate::cubicBezierSecondDerivative(_p0[0], _pf[0], phase) / (swingTime * swingTime);
        _p[1] = Interpolate::cubicBezier(_p0[1], _pf[1], phase);
        _v[1] = Interpolate::cubicBezierFirstDerivative(_p0[1], _pf[1], phase);
        _a[1] = Interpolate::cubicBezierSecondDerivative(_p0[1], _pf[1], phase) / (swingTime * swingTime);
        _p[2] = Interpolate::cubicBezier(_p0[2], _pf[2], phase);
        _v[2] = Interpolate::cubicBezierFirstDerivative(_p0[2], _pf[2], phase);
        _a[2] = Interpolate::cubicBezierSecondDerivative(_p0[2], _pf[2], phase) / (swingTime * swingTime);

        double zp, zv, za;

        if(phase < 0.5) {
            zp = Interpolate::cubicBezier(_p0[2], _p0[2] + _height, phase * 2);
            zv = Interpolate::cubicBezierFirstDerivative(_p0[2], _p0[2] + _height, phase * 2);
            za = Interpolate::cubicBezierSecondDerivative(_p0[2], _p0[2] + _height, phase * 2) * 4 / (swingTime * swingTime);
        } else {
            zp = Interpolate::cubicBezier(_p0[2] + _height, _pf[2], phase * 2 - 1);
            zv = Interpolate::cubicBezierFirstDerivative(_p0[2] + _height, _pf[2], phase * 2 - 1) ;
            za = Interpolate::cubicBezierSecondDerivative(_p0[2] + _height, _pf[2], phase * 2 - 1) * 4 / (swingTime * swingTime);
        }

        _p[2] = zp;
        _v[2] = zv;
        _a[2] = za;
    }

    /*!
    * Compute foot swing trajectory with a bezier curve
    * @param phase : How far along we are in the swing (0 to 1)
    * @param swingTime : How long the swing should take (seconds)
    */
    void computeStanceTrajectoryBezier(double phase, double swingTime) {
        _p[0] = Interpolate::cubicBezier(_p0[0], _pf[0], phase);
        _v[0] = Interpolate::cubicBezierFirstDerivative(_p0[0], _pf[0], phase);
        _a[0] = Interpolate::cubicBezierSecondDerivative(_p0[0], _pf[0], phase) / (swingTime * swingTime);
        _p[1] = Interpolate::cubicBezier(_p0[1], _pf[1], phase);
        _v[1] = Interpolate::cubicBezierFirstDerivative(_p0[1], _pf[1], phase);
        _a[1] = Interpolate::cubicBezierSecondDerivative(_p0[1], _pf[1], phase) / (swingTime * swingTime);
        _p[2] = Interpolate::cubicBezier(_p0[2], _pf[2], phase);
        _v[2] = Interpolate::cubicBezierFirstDerivative(_p0[2], _pf[2], phase);
        _a[2] = Interpolate::cubicBezierSecondDerivative(_p0[2], _pf[2], phase) / (swingTime * swingTime);
    }

    /*!
     * Get the foot position at the current point along the swing
     * @return : the foot position
     */
    Vec3 getPosition() {
        return _p;
    }

    /*!
     * Get the foot velocity at the current point along the swing
     * @return : the foot velocity
     */
    Vec3 getVelocity() {
        return _v;
    }

    /*!
     * Get the foot acceleration at the current point along the swing
     * @return : the foot acceleration
     */
    Vec3 getAcceleration() {
        return _a;
    }
};


#endif
