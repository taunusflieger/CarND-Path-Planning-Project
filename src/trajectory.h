#pragma once

#include "config.h"
#include "jmt.h"
#include "map.h"
#include "types.h"
#include "vehicle.h"
#include <vector>

class Trajectory {

public:
  Trajectory(Config &cfg, Map &map) : cfg_(cfg), map_(map){};
  Trajectory(Vehicle &car, const BehaviorType behavior, Map &map, Config &cfg);
  TrajectoryXY generateJMTTrajectory(Vehicle &car, const BehaviorType behavior,
                                     int numPoints);
  TrajectoryXY generateSplineBasedTrajectory(Vehicle &car,
                                             TrajectoryXY &previous_path);
  TrajectoryXY generateTrajectory(Vehicle &car, const BehaviorType behavior,
                                  TrajectoryXY &previous_path, int numPoints);
  TrajectoryJMT generateColdStartPrevPath(double s, double d);
  State targetState_d;
  State targetState_s;
  JMT get_jmt_s() const;
  JMT get_jmt_d() const;

private:
  vector<JMT> jmtPair_;
  Config &cfg_;
  Map &map_;
};
