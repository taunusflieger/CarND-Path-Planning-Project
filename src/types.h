#pragma once

#include <iostream>
#include <vector>

// the d value of each lane's center
const double LEFT_d = 2.2;
const double MID_d = 6.0;
const double RIGHT_d = 9.8;

struct CarLocalizationData {
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double speed;

  CarLocalizationData(double x = 0, double y = 0, double s = 0, double d = 0,
                      double yaw = 0, double speed = 0)
      : x(x), y(y), s(s), d(d), yaw(yaw), speed(speed) {}
};

struct SensorFusionData {
  int id;
  double x;
  double y;
  double vx;
  double vy;
  double s;
  double d;
};

/* State - stores three doubles p, v, a
 * intended to store position, velocity, and acceleration components in the s,
 * or d axis
 */
struct State {
  double p;
  double v;
  double a;
};

enum class LaneType { LEFT, MID, RIGHT, NONE, UNSPECIFIED };

enum class BehaviorType { KEEPLANE, TURNRIGHT, TURNLEFT };

// used to stores the s or d component of a frenet point and its derivatives
// we don't store t as we are calculating all algorithim with a fixed dt
struct PointCmp {
  // value - either d or s
  double v;
  // 1st derivative
  double v_dot;
  // 2nd derivative
  double v_ddot;
  PointCmp(double V = 0, double V_dot = 0, double V_ddot = 0)
      : v(V), v_dot(V_dot), v_ddot(V_ddot) {}
};

/* XYPoints stores two vectors x, y which are the map coordinates to be passed
 * to the simulator. Also holds an int n intended to store the number of (x, y)
 * pairs
 */
struct XYPoints {
  std::vector<double> xs;
  std::vector<double> ys;
  int n;
};

struct TrajectoryXY {
  XYPoints pts;
  TrajectoryXY(std::vector<double> XPts, std::vector<double> YPts) {
    pts.xs = XPts;
    pts.ys = YPts;
    pts.n = XPts.size();
  };
  TrajectoryXY(){};
};

struct TrajectorySD {
  std::vector<PointCmp> path_s;
  std::vector<PointCmp> path_d;
  TrajectorySD(std::vector<PointCmp> S = {}, std::vector<PointCmp> D = {})
      : path_s(S), path_d(D) {}
};

struct TrajectoryJMT {
  TrajectoryXY trajectory;
  TrajectorySD path_sd;
};

struct PreviousPath {
  TrajectoryXY xy;
  TrajectorySD sd;
  PreviousPath(TrajectoryXY XY = {}, TrajectorySD SD = {}, int N = 0)
      : xy(XY), sd(SD){};
};
