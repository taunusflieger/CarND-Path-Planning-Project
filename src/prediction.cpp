// =================================================================
//  Generates situational awareness by predicting the behavior of
//  the other cars within a given planning horizon
// =================================================================

#include "prediction.h"
#include <map>
#include <iostream>
#include "log.h"

extern Logger log_;
using namespace std;


Prediction::Prediction(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                       int planning_horizon, Config &cfg)
    : egoCar_(egoCar), cfg_(cfg) {
  // std::map<int, vector<Coord> > predictions;
}

std::vector<int>
Prediction::getNearbyCars(std::vector<Vehicle>  &otherCars) {
  std::vector<double> distance_back_object = {INFINITY, INFINITY, INFINITY};
  std::vector<double> distance_front_object = {INFINITY, INFINITY, INFINITY};

  // Define range to search for nearby cars
  // Handle FOV and s wraparound
  double sfov_min = egoCar_.s - cfg_.sensorRangeBack();
  double sfov_max = egoCar_.s + cfg_.sensorRangeFront();
  double sfov_wrap = 0;

  // Check if we are at the end of the track, if so
  // need to handle wraparound
  if (sfov_min < 0) {
    sfov_wrap = -sfov_min;
  } else if (sfov_max > cfg_.trackLength()) {
    sfov_wrap = cfg_.trackLength() - sfov_max;
  }

  sfov_min += sfov_wrap;
  sfov_max += sfov_wrap;
  assert(sfov_min >= 0 && sfov_min <= cfg_.trackLength());
  assert(sfov_max >= 0 && sfov_max <= cfg_.trackLength());
  // log_.write("Prediction::getNearbyCars =================");

  for (size_t i = 0; i < otherCars.size(); i++) {
    double s = otherCars[i].s + sfov_wrap;
    double dist = fabs(s - egoCar_.s + sfov_wrap);
    
    // Check if other car is in sensor range
    if (s >= sfov_min && s <= sfov_max) {
      LaneType lane = otherCars[i].convert_d_to_lane();
      
      if (lane == LaneType::UNSPECIFIED)
        continue; // ignore error from simulator

      // og_.of_ << "other car id = " << otherCars[i].id << "\tlane = " << static_cast<int>(lane) << "\tvelocity = " << otherCars[i].v;

      // if is in front of the ergCar and the distance to it is mall than
      // the distance of a car we already have recorded
      if (s > (egoCar_.s + sfov_wrap) &&
          (dist < distance_front_object[static_cast<int>(lane)])) {
        nearby_front_[static_cast<int>(lane)] = i;
        distance_front_object[static_cast<int>(lane)] = dist;
        otherCars[i].front_gap = dist;
        // log_.of_ << "\tfront dist = " << dist << endl;
      } else if (dist < distance_back_object[static_cast<int>(lane)]) {
        nearby_back_[static_cast<int>(lane)] = i;
        distance_back_object[static_cast<int>(lane)] = dist;
        // log_.of_  << "\tback dist = " << dist << endl;
      }
    }
  }
  /* 
  log_.of_ << nearby_front_[0] << " , " << nearby_back_[0] << endl
           << nearby_front_[1] << " , " << nearby_back_[1] << endl
           << nearby_front_[2] << " , " << nearby_back_[2] << endl;

 */  // log_.write("Prediction::getNearbyCars =================");
  //
  //

  return {nearby_front_[0], nearby_back_[0], nearby_front_[1], nearby_back_[1], nearby_front_[2], nearby_back_[2]};
}



