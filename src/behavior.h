#pragma once

#include "types.h"

struct Target {
  double velocity;
  LaneType lane;

  // Calculated time to reach the target position
  double time;
  Target(LaneType l = LaneType::UNSPECIFIED, double v = 0, double t = 0)
      : lane(l), velocity(v), time(t) {}
};

class Behavior {
public:
  Behavior(){};
};