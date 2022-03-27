/*! @file Interpolation.h
 *  @brief Utility functions to interpolate between two values
 *
 */

#ifndef _INTERPOLATION_
#define _INTERPOLATION_

#include <assert.h>
#include <type_traits>

namespace Interpolate {

/*!
 * Linear interpolation between y0 and yf.  x is between 0 and 1
 */
double Lerp(double y0, double yf, double x) {
  static_assert(std::is_floating_point<double>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  return y0 + (yf - y0) * x;
}

/*!
 * Cubic bezier interpolation between y0 and yf.  x is between 0 and 1
 */
double cubicBezier(double y0, double yf, double x) {
  static_assert(std::is_floating_point<double>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  double yDiff = yf - y0;
  double bezier = x * x * x + double(3) * (x * x * (double(1) - x));
  return y0 + bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf. x is between 0 and 1
 */
double cubicBezierFirstDerivative(double y0, double yf, double x) {
  static_assert(std::is_floating_point<double>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  double yDiff = yf - y0;
  double bezier = double(6) * x * (double(1) - x);
  return bezier * yDiff;
}

/*!
 * Cubic bezier interpolation derivative between y0 and yf.  x is between 0 and 1
 */
double cubicBezierSecondDerivative(double y0, double yf, double x) {
  static_assert(std::is_floating_point<double>::value,
                "must use floating point value");
  assert(x >= 0 && x <= 1);
  double yDiff = yf - y0;
  double bezier = double(6) - double(12) * x;
  return bezier * yDiff;
}

}  // namespace Interpolate

#endif  // _INTERPOLATION_
