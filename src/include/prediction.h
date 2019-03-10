#pragma once
#include <math.h>
#include <vector>
#include "types.h"
#include "vehicle.h"
#include "config.h"
#include "trajectory.h"

struct LaneTrajectories {
  bool front_valid = false;
  bool back_valid = false;
  TrajectoryJMT front_jmt;
  TrajectoryJMT back_jmt;
};

class Prediction {
 public:
  Prediction(std::vector<Vehicle>  &otherCars, Vehicle  &egoCar,
             int planning_horizon, Map &map, Config  &cfg);

  std::vector<int> getNearbyFrontCars();
  std::vector<int> getNearbyCars(std::vector<Vehicle>  &otherCars);
  bool IsTrajectoryCollisionFree(TrajectoryCandidate &tc);

 private:
  Config& cfg_;
  Vehicle& egoCar_;
  std::map<int, LaneTrajectories> predictions_;

  // For each lane we store the index of the
  // car each is near by. If there is no car in range
  // we indicate this through the value -1
  std::vector<int> nearby_front_ = {-1, -1, -1};
  std::vector<int> nearby_back_ = {-1, -1, -1};
  bool contains(double n, vector<double> range);
  bool overlap(vector<double> a, vector<double> b);
  bool checkIndividualOtherCar(XYPoints &pts, TrajectoryCandidate &tc);
  bool checkCollision(double s0, double d0, double theta0, double s1, double d1,
                      double theta1);
};