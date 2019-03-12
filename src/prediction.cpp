// =================================================================
//  Generates situational awareness by predicting the behavior of
//  the other cars within a given planning horizon
// =================================================================
// clang-format off
#include "prediction.h"
//#include <map>
#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "log.h"
#include <cassert>

// clang-format on

extern Logger log_;

Prediction::Prediction(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                       int planning_horizon, Map &map, Config &cfg)
    : egoCar_(egoCar), cfg_(cfg) {
  std::vector<int> nearbyCars;

  log_.write("==== BEGIN Prediction::Prediction ====");
  nearbyCars = getNearbyCars(otherCars);

  for (int i = 0; i < 3; i++) {
    LaneTrajectories laneTraj;
    laneTraj.front_valid = false;
    laneTraj.back_valid = false;
    predictions_[i] = laneTraj;
  }

  // Calculate trajectories for nearby cars on each lane
  for (int i = 0; i < nearbyCars.size(); i++) {
    Target t;
    PreviousPath prev_path;  // is empty on purpose as we don't have the data

    int car_idx = nearbyCars[i];
    if (car_idx > -1) {
      int lane = static_cast<int>(otherCars[car_idx].lane);
      log_.of_ << "creating others car traj for lane = " << lane << endl;
      t.velocity = otherCars[car_idx].v;
      t.time = cfg_.traverseTime();
      t.lane = otherCars[car_idx].lane;
      t.acceleration = 1;  // we assume the other cars don't change speed

      log_.of_ << "Other car idx = " << car_idx << " car_id = " << otherCars[car_idx].id << " lane = " << static_cast<int>(otherCars[car_idx].lane) << " font distance = " << otherCars[car_idx].front_gap << endl;

      Trajectory trajectory(cfg_, map);
      if (i % 2 != 0) {
        predictions_[lane].back_jmt = trajectory.generateSDTrajectory(otherCars[car_idx], prev_path, t);
        predictions_[lane].back_valid = true;
        log_.of_ << "Trajectory generated for nearby car lane = " << lane << " position: back" << endl;
      } else {
        predictions_[lane].front_jmt = trajectory.generateSDTrajectory(otherCars[car_idx], prev_path, t);
        predictions_[lane].front_valid = true;
        log_.of_ << "Trajectory generated for nearby car lane = " << lane << " position: front" << endl;
      }
    }
  }
  log_.write("==== END Prediction::Prediction ====");
}

std::vector<int>
Prediction::getNearbyFrontCars() {
  return {
      nearby_front_[0],
      nearby_front_[1],
      nearby_front_[2],
  };
}

std::vector<int> Prediction::getNearbyCars(std::vector<Vehicle> &otherCars) {
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
  log_.write("===== BEGIN Prediction::getNearbyCars =====");

  for (size_t i = 0; i < otherCars.size(); i++) {
    double s = otherCars[i].s + sfov_wrap;
    double dist = fabs(s - egoCar_.s + sfov_wrap);

    // Check if other car is in sensor range
    if (s >= sfov_min && s <= sfov_max) {
      int lane = static_cast<int>(otherCars[i].convert_d_to_lane());

      if (lane == static_cast<int>(LaneType::UNSPECIFIED))
        continue;  // ignore error from simulator

      log_.of_ << "other_car idx = " << i << "\tother car id = " << otherCars[i].id << "\tlane = " 
               << static_cast<int>(lane) << " dist = " << dist << "\tvelocity = " << otherCars[i].v 
               << " s = " << s << " egoCar_.s = " << egoCar_.s << " sfov_wrap = " << sfov_wrap 
               << " distance_front_object[lane] = " 
               << distance_front_object[lane];

      // if is in front of the ergCar and the distance to it is smaller than
      // the distance of a car we already have recorded
      if (s > (egoCar_.s + sfov_wrap) && (dist < distance_front_object[lane])) {
        nearby_front_[lane] = i;
        distance_front_object[lane] = dist;
        otherCars[i].front_gap = dist;
        log_.of_ << "\tfront dist = " << dist << endl;
      } 
      else if (dist < distance_back_object[lane]) {
        nearby_back_[lane] = i;
        distance_back_object[lane] = dist;
        log_.of_ << "\tback dist = " << dist << endl;
      }
      else 
        log_.of_ << endl;
    }
  }
  log_.of_ << "=============================================" << endl;
  log_.of_ << "lane = 0: " << (nearby_front_[0] > 0 ? otherCars[nearby_front_[0]].id : -1) << " , " << (nearby_back_[0] > 0 ? otherCars[nearby_back_[0]].id : -1) << endl
           << "lane = 1: " << (nearby_front_[1] > 0 ? otherCars[nearby_front_[1]].id : -1) << " , " << (nearby_back_[1] > 0 ? otherCars[nearby_back_[1]].id : -1) << endl
           << "lane = 2: " << (nearby_front_[2] > 0 ? otherCars[nearby_front_[2]].id : -1) << " , " << (nearby_back_[2] > 0 ? otherCars[nearby_back_[2]].id : -1) << endl;
  log_.of_ << "=============================================" << endl;
  log_.write("==== END Prediction::getNearbyCars =====");

  return {nearby_front_[0], nearby_back_[0], nearby_front_[1], nearby_back_[1], nearby_front_[2], nearby_back_[2]};
}

// Check a trajectory candidate for collision risk with other cars
bool Prediction::IsTrajectoryCollisionFree(TrajectoryCandidate &tc) {
  log_.of_ << "==== Prediction::IsTrajectoryCollisionFree ====" << endl;
  bool front = true, back = true;
  int lane = static_cast<int>(tc.t.lane);
  XYPoints front_xy = predictions_[lane].front_jmt.trajectory.pts;
  XYPoints back_xy = predictions_[lane].back_jmt.trajectory.pts;

  /* log_.of_ << "front x size = " << front_xy.xs.size() << endl;
  log_.of_ << "back x size = " << back_xy.xs.size() << endl; */
  log_.of_ << "Check lane = " << lane << endl;
  if (predictions_[lane].front_valid) {
    front = checkIndividualOtherCar(front_xy, tc);
    if (front)
      log_.of_ << "Check lane = " << lane << " font: no collision" << endl;
    else
      log_.of_ << "Check lane = " << lane << " font: COLLISION" << endl;
  }
  else
  {
    log_.of_ << "Check lane = " << lane << " NO TRAJ DATA FOR FRONT" << endl;
  }
  

  if (predictions_[lane].back_valid) {
    back = checkIndividualOtherCar(back_xy, tc);
    if (back)
      log_.of_ << "Check lane = " << lane << " back: no collision" << endl;
    else
      log_.of_ << "Check lane = " << lane << " back: COLLISION" << endl;
  }
  else
  {
    log_.of_ << "Check lane = " << lane << " NO TRAJ DATA FOR BACK" << endl;
  }

  // *** REMOVE
  if (front && back)
    log_.of_ << "No collision detected" << endl;
  else
    log_.of_ << "Collision detected" << endl;

  log_.of_ << "==== ****** ====" << endl;
  return (front && back);
}

// Check for an individual other car if a collision can occur
bool Prediction::checkIndividualOtherCar(XYPoints &pts, TrajectoryCandidate &tc) {
  assert(pts.xs.size() >= cfg_.planAhead());

  // Check for collision during the time span of the lane change
  for (int i = 0; i < (int)(cfg_.traverseTime() / 0.02); i++) {
    double obj_x = pts.xs[i];
    double obj_y = pts.ys[i];
    double obj_x_next = pts.xs[i + 1];
    double obj_y_next = pts.ys[i + 1];
    double obj_heading = atan2(obj_y_next - obj_y, obj_x_next - obj_x);

    double ego_x = tc.jmt_traj.trajectory.pts.xs[i];
    double ego_y = tc.jmt_traj.trajectory.pts.ys[i];
    double ego_x_next = tc.jmt_traj.trajectory.pts.xs[i + 1];
    double ego_y_next = tc.jmt_traj.trajectory.pts.xs[i + 1];
    double ego_heading = atan2(ego_y_next - ego_y, ego_x_next - ego_x);

    if (checkCollision(obj_x, obj_y, obj_heading, ego_x, ego_y, ego_heading)) 
      return false;
  }
  return true;
}

bool Prediction::contains(double n, vector<double> range) {
  float a = range[0], b = range[1];
  if (b < a) {
    a = b;
    b = range[0];
  }
  return (n >= a && n <= b);
}

bool Prediction::overlap(vector<double> a, vector<double> b) {
  if (contains(a[0], b)) return true;
  if (contains(a[1], b)) return true;
  if (contains(b[0], a)) return true;
  if (contains(b[1], a)) return true;
  return false;
}

// Uses the separation of axis theorem to detect collisions
bool Prediction::checkCollision(double s0, double d0, double theta0, double s1, double d1, double theta1) {
  
  // set safety distance (to vehicle heading)
  double safety_dist_lon = cfg_.carSafetyLength();
  double safety_dist_lat = cfg_.carSafetyWidth();

  // vehicle wrapper
  Eigen::MatrixXd rec_wrapper(2, 4);
  rec_wrapper << safety_dist_lon, safety_dist_lon, -safety_dist_lon, -safety_dist_lon,
      -safety_dist_lat, safety_dist_lat, safety_dist_lat, -safety_dist_lat;

  // rotate wrapper by heading
  Eigen::Matrix2d rot0, rot1;
  rot0 << cos(theta0), -sin(theta0), sin(theta0), cos(theta0);
  rot1 << cos(theta1), -sin(theta1), sin(theta1), cos(theta1);

  Eigen::MatrixXd rec0(2, 4);
  Eigen::MatrixXd rec1(2, 4);
  Eigen::Vector2d trans0, trans1;
  trans0 << s0, d0;
  trans1 << s1, d1;

  for (int i = 0; i < rec_wrapper.cols(); i++) {
    rec0.col(i) = rot0 * rec_wrapper.col(i) + trans0;
    rec1.col(i) = rot1 * rec_wrapper.col(i) + trans1;
  }

  // set principal axis list: normal + normal perpendicular
  Eigen::MatrixXd axis(2, 4);
  axis << cos(theta0), sin(theta0), cos(theta1), sin(theta1),
      sin(theta0), -cos(theta0), sin(theta1), -cos(theta1);

  for (int i = 0; i < axis.cols(); i++) {
    Eigen::Vector2d principal_axis = axis.col(i);
    // projection of rec0: get min, max
    double min0 = principal_axis.dot(rec0.col(0));
    double max0 = min0;
    for (int j = 0; j < rec0.cols(); j++) {
      double proj0 = principal_axis.dot(rec0.col(j));
      if (proj0 > max0) max0 = proj0;
      if (proj0 < min0) min0 = proj0;
    }
    // projection of rec1: get min, max
    double min1 = principal_axis.dot(rec1.col(0));
    double max1 = min1;
    for (int j = 0; j < rec1.cols(); j++) {
      double proj1 = principal_axis.dot(rec1.col(j));
      if (proj1 > max1) max1 = proj1;
      if (proj1 < min1) min1 = proj1;
    }
    // check overlap
    if (!overlap({min0, max0}, {min1, max1})) return false;
  }
  return true;
}
