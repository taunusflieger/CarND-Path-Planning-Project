#include "vehicle.h"

Vehicle::Vehicle(const int i, Config &cfg) : id(i), cfg_(cfg) {
  lane = LaneType::UNSPECIFIED;
}

void Vehicle::update_position(const double s, const double d) {
  this->s = s;
  this->d = d;
  this->lane = this->convert_d_to_lane(this->d);
}

void Vehicle::update_speed(const double v) { this->v = v; }

void Vehicle::update_save_states(const State &state_s, const State &state_d) {
  this->saved_state_s = state_s;
  this->saved_state_d = state_d;
  this->saved_state_s.p = fmod(this->saved_state_s.p, cfg_.trackLength());
}

void Vehicle::specify_adjacent_lanes() {

  if (this->lane == LaneType::LEFT) {

    this->lane_at_left = LaneType::NONE;
    this->lane_at_right = LaneType::MID;

  } else if (this->lane == LaneType::MID) {

    this->lane_at_left = LaneType::LEFT;
    this->lane_at_right = LaneType::RIGHT;

  } else if (this->lane == LaneType::RIGHT) {

    this->lane_at_left = LaneType::MID;
    this->lane_at_right = LaneType::NONE;

  } else {

    this->lane = LaneType::UNSPECIFIED;
    this->lane_at_left = LaneType::UNSPECIFIED;
    this->lane_at_right = LaneType::UNSPECIFIED;
  }
}

LaneType Vehicle::convert_d_to_lane(const double d) {

  LaneType lane = LaneType::NONE;

  if (d > 0.0 && d < 4.0) {
    lane = LaneType::LEFT;
  } else if (d > 4.0 && d < 8.0) {
    lane = LaneType::MID;
  } else if (d > 8.0 && d < 12.0) {
    lane = LaneType::RIGHT;
  }
  return lane;
}

LaneType Vehicle::convert_d_to_lane() {
  return this->convert_d_to_lane(this->d);
}

double Vehicle::convert_lane_to_d(const LaneType l) {

  double d = MID_d;

  if (l == LaneType::LEFT) {
    d = LEFT_d;
  } else if (l == LaneType::MID) {
    d = MID_d;
  } else if (l == LaneType::RIGHT) {
    d = RIGHT_d;
  }
  return d;
}

double Vehicle::convert_lane_to_d() {
  return this->convert_lane_to_d(this->lane);
}

double Vehicle::get_target_d(const BehaviorType b) {

  if (b == BehaviorType::KEEPLANE) {
    return this->convert_lane_to_d(this->lane);

  } else if (b == BehaviorType::TURNRIGHT) {
    return this->convert_lane_to_d(this->lane_at_right);

  } else if (b == BehaviorType::TURNLEFT) {
    return this->convert_lane_to_d(this->lane_at_left);
  }

  return this->convert_lane_to_d(this->lane);
}

TrajectoryXY Vehicle::startEngine(Map &map) {

  const int n = 225;
  const double t = n * cfg_.timeIncrement();
  const double target_speed = 20.0;
  const double target_s = s + 40.0;

  const State startState_s = {s, v, 0.0};
  const State startState_d = {d, 0.0, 0.0};

  const State endState_s = {target_s, target_speed, 0.0};
  const State endState_d = {convert_lane_to_d(), 0.0, 0.0};

  JMT jmt_s(startState_s, endState_s, t);
  JMT jmt_d(startState_d, endState_d, t);

  update_save_states(endState_s, endState_d);

  XYPoints pts = map.makePath(jmt_s, jmt_d, cfg_.timeIncrement(), n);
  TrajectoryXY new_traj(pts.xs, pts.ys);

  return new_traj;
}

double Vehicle::getTargetSpeed() { return 0.0; }