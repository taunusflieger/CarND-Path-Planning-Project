#pragma once

#include "config.h"
#include "map.h"
#include "types.h"
#include <iostream>
#include <math.h>
#include <vector>

class Vehicle {
private:
  Config &cfg_;

public:
  int id;
  double x;
  double y;
  double s;
  double d;
  double v;
  double yaw;
  double front_gap;
  double front_v;
  double front_s;

  State saved_state_s;
  State saved_state_d;

  LaneType lane;
  LaneType lane_at_right;
  LaneType lane_at_left;

  Vehicle(const int i, Config &cfg);
  void updatePositionXY(double X, double Y) {
    x = X;
    y = Y;
  };
  void updateYaw(double Yaw) { yaw = Yaw; };
  void update_position(const double s, const double d);
  void update_speed(const double v);
  void update_save_states(const State &state_s, const State &state_d);
  void specify_adjacent_lanes();

  LaneType convert_d_to_lane(const double d) const;
  LaneType convert_d_to_lane() const;
  double convert_lane_to_d(const LaneType l);
  double convert_lane_to_d();
  double get_target_d(const BehaviorType b);
  double getTargetSpeed();
  TrajectoryXY startEngine(Map &map);
};
