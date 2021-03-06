#include <math.h>
#include <iostream>
#include <vector>
#include "log.h"
#include "cost.h"

using namespace std;

extern Logger log_;

Cost::Cost(Config &cfg) : cfg_(cfg) {}

double Cost::getCost(TrajectoryCandidate& tc, Prediction &predictions, std::vector<Vehicle> &otherCars, Vehicle &egoCar) {
  double cost = 0;
  log_.write("==== START Cost::getCost ====");

  // Check if change to target lane will lead to a collision
  if (!predictions.IsTrajectoryCollisionFree(tc))
    cost += 10000;

  // Add speed difference between ego car and other car on the target
  // as a cost factor. This will allow selection of fastest lane for change
  int lane = static_cast<int>(tc.t.lane);
  std::vector<int> nearbyCars = predictions.getNearbyFrontCars();
  int car_idx = nearbyCars[lane];
  log_.of_ << "Check cost for lane = " << lane << endl;
  if (car_idx > 0) {
    Vehicle other_car = otherCars[car_idx];
    double vd = egoCar.v  - other_car.v;
    log_.of_ << "Speed difference on target lane = " <<lane << " v = " << vd  << endl;
    if (vd < 0)
      cost += 1000;
    else {
      // If a car in front of the target lane is too close, a lane change
      // is considered to be unsafe
      log_.of_ << "Check for unsafe lange change: other car front gap = " << other_car.front_gap << " vd = " << vd << endl;
      if (other_car.front_gap < cfg_.collisionBuffer()) {
        cost += 10000;
        log_.of_ << "Speed difference too high and front car too close: UNSAFE LANE CHANGE DETECTED" << endl;
      } 
      else
        cost += vd;
    }  
  }
  log_.write("==== END Cost::getCost ====");
  return cost;
}
