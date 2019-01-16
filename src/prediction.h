#pragma once

#include <math.h>
#include <vector>
#include "types.h"

class Prediction {
  public:
    Prediction() {};

  private:
    double dist_front_;
    double vel_front_;
    double time_to_collision_;  // vs front vehicle
    double time_to_stop_;  // time from vel_ego_ to 0
};