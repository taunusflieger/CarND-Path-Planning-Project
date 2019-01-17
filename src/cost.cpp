
// clang-format off
#include <math.h>
#include <iostream>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "cost.h"
// clang-format on

using namespace std;
using namespace Eigen;

Cost::Cost(Config &cfg) : cfg_(cfg) {}

// checkCollision implements SAT (Separating Axis Theorem)
// to identify collision between 2 convex rectangular objects
// http://www.dyn4j.org/2010/01/sat/
bool Cost::checkCollision(double s0, double d0, double theta0, double s1,
                          double d1, double theta1) {

  // set safety distance (to vehicle heading)
  double safety_dist_lon = cfg_.carSafetyLength();
  double safety_dist_lat = cfg_.carSafetyWidth();

  // vehicle wrapper
  MatrixXd rec_wrapper(2, 4);
  rec_wrapper << safety_dist_lon, safety_dist_lon, -safety_dist_lon,
      -safety_dist_lon, -safety_dist_lat, safety_dist_lat, safety_dist_lat,
      -safety_dist_lat;

  // rotate wrapper by heading
  Matrix2d rot0, rot1;
  rot0 << cos(theta0), -sin(theta0), sin(theta0), cos(theta0);
  rot1 << cos(theta1), -sin(theta1), sin(theta1), cos(theta1);

  MatrixXd rec0(2, 4);
  MatrixXd rec1(2, 4);
  Vector2d trans0, trans1;
  trans0 << s0, d0;
  trans1 << s1, d1;

  for (int i = 0; i < rec_wrapper.cols(); i++) {
    rec0.col(i) = rot0 * rec_wrapper.col(i) + trans0;
    rec1.col(i) = rot1 * rec_wrapper.col(i) + trans1;
  }

  // set principal axis list: normal + normal perpendicular
  MatrixXd axis(2, 4);
  axis << cos(theta0), sin(theta0), cos(theta1), sin(theta1), sin(theta0),
      -cos(theta0), sin(theta1), -cos(theta1);

  for (int i = 0; i < axis.cols(); i++) {
    Vector2d principal_axis = axis.col(i);
    // projection of rec0: get min, max
    double min0 = principal_axis.dot(rec0.col(0));
    double max0 = min0;
    for (int j = 0; j < rec0.cols(); j++) {
      double proj0 = principal_axis.dot(rec0.col(j));
      if (proj0 > max0)
        max0 = proj0;
      if (proj0 < min0)
        min0 = proj0;
    }
    // projection of rec1: get min, max
    double min1 = principal_axis.dot(rec1.col(0));
    double max1 = min1;
    for (int j = 0; j < rec1.cols(); j++) {
      double proj1 = principal_axis.dot(rec1.col(j));
      if (proj1 > max1)
        max1 = proj1;
      if (proj1 < min1)
        min1 = proj1;
    }
    // check overlap
    return (min1 >= min0 && min1 < max0) || (max1 >= min0 && max1 < max0) ||
           (min0 >= min1 && min0 < max1) || (max0 >= min1 && max0 < max1);
  }
  return true;
}
