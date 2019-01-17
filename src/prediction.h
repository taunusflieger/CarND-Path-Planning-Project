#pragma once
#include <math.h>
#include <vector>
#include "types.h"
#include "vehicle.h"
#include "config.h"


class Prediction {
public:
  Prediction(std::vector<Vehicle>  &otherCars, Vehicle  &egoCar,
             int planning_horizon, Config  &cfg);

private:
  std::vector<int> getNearbyCars(std::vector<Vehicle> const &otherCars);

  // For each lane we store the index of the
  // car each is near by. If there is no car in range
  // we indicate this through the value -1
  std::vector<int> nearby_front_ = {-1, -1, -1};
  std::vector<int> nearby_back_ = {-1, -1, -1};
  
  

  Config& cfg_;
  Vehicle& egoCar_;
  double dist_front_;
  double vel_front_;
  double time_to_collision_; // vs front vehicle
  double time_to_stop_;      // time from vel_ego_ to 0
};