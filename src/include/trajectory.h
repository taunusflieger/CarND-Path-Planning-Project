#pragma once

#include <vector>
#include "config.h"
#include "jmt.h"
#include "map.h"
#include "types.h"
#include "vehicle.h"

class Trajectory {
 public:
  Trajectory(Config &cfg, Map &map) : cfg_(cfg), map_(map){};
  Trajectory(Vehicle &car, const BehaviorType behavior, Map &map, Config &cfg);
  TrajectoryXY generateJMTTrajectory(Vehicle &car, const BehaviorType behavior,
                                     int numPoints);
  TrajectoryXY generateSplineBasedTrajectory(Vehicle &car, Target &target,
                                             TrajectoryXY &previous_path);
  TrajectoryXY generateTrajectory(Vehicle &car, const BehaviorType behavior,
                                  TrajectoryXY &previous_path, Target &target);

  TrajectoryJMT generateSDTrajectory(Vehicle &car,
                                     PreviousPath &previous_path,
                                     Target &target);
  TrajectoryJMT JMT_init(double car_s, double car_d);
  
  State targetState_d;
  State targetState_s;
  JMT get_jmt_s() const;
  JMT get_jmt_d() const;

 private:
  vector<JMT> jmtPair_;
  Config &cfg_;
  Map &map_;
};
