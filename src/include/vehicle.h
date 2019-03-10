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
  double vx;
  double vy;
  double yaw;
  double front_gap;
  LaneType lane;
  
  Vehicle(SensorFusionData& sf, Config &cfg);
  Vehicle(int id, Config &cfg) : id(id), cfg_(cfg){};
  void updatePositionXY(double X, double Y) {
    x = X;
    y = Y;
  };
  void updateYaw(double Yaw) { yaw = Yaw; };
  void update_position(const double s, const double d);
  void update_speed(const double v);

  LaneType convert_d_to_lane(const double d) const;
  LaneType convert_d_to_lane() const;
  double convert_lane_to_d(const LaneType l);
  double convert_lane_to_d();
};
