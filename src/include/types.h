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
  double v;

  SensorFusionData(int id, double x, double y, double vx, double vy, double s,
                   double d)
      : id(id), x(x), y(y), vx(vx), vy(vy), s(s), d(d) {}
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

enum class LaneType : int {
  LEFT = 0,
  MID = 1,
  RIGHT = 2,
  NONE = 98,
  UNSPECIFIED = 99
};

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

struct XYCoord {
  double x;
  double y;
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


// Defines the target position of the car at a specific time in the future.
// This is used during behavior planning and trajectory generation
struct Target {
  double velocity;
  double acceleration;
  double s;
  // Calculated time to reach the target position
  double time;
  LaneType lane;

  Target(LaneType l = LaneType::UNSPECIFIED, double v = 0, double acceleration = 0, double s = 0, double t = 0)
      : lane(l), velocity(v), acceleration(acceleration), s(s), time(t) {}
};

struct TrajectoryCandidate {
  Target t;
  TrajectoryJMT jmt_traj;
  double cost;
};

struct PreviousPath {
  TrajectoryXY xy;
  TrajectorySD sd;
  int num_xy_reused;
  PreviousPath(TrajectoryXY xy = {}, TrajectorySD sd = {}, int n = 0)
      : xy(xy), sd(sd), num_xy_reused(n) {};
};

