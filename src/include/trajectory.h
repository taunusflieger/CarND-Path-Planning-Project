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

    TrajectoryJMT generateSDTrajectory(Vehicle &car,
                                       PreviousPath &previous_path,
                                       Target &target);

    TrajectoryJMT generate_trajectory_jmt(Target target, Map &map, PreviousPath const &previous_path);
    TrajectoryJMT JMT_init(double car_s, double car_d);
 
   private:
    double polyeval(vector<double> c, double t);
    double polyeval_dot(vector<double> c, double t);
    double polyeval_ddot(vector<double> c, double t);
    double getDPosition(LaneType lane);
    Config &cfg_;
    Map &map_;
};
