#pragma once

#include "types.h"
#include "prediction.h"


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
  Behavior(std::vector<Vehicle>&  otherCars, Vehicle& egoCar,
             Prediction& predictions, Config& cfg);

private:
  Config& cfg_;
  Vehicle& egoCar_;  
};