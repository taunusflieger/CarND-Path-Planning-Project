#include "vehicle.h"

Vehicle::Vehicle(SensorFusionData& sf, Config &cfg) : cfg_(cfg) {
  lane = LaneType::UNSPECIFIED;

  id = sf.id;
  x = sf.x;
  y = sf.y;
  s = sf.s;
  d = sf.d;
  vx = sf.vx;
  vy = sf.vy;
  v = sqrt(vx * vx + vy * vy);
}

void Vehicle::update_position(const double s, const double d) {
  this->s = s;
  this->d = d;
  this->lane = this->convert_d_to_lane(this->d);
}

void Vehicle::update_speed(const double V) { v = V; }

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

LaneType Vehicle::convert_d_to_lane(const double d) const {

  LaneType lane = LaneType::NONE;

  if (d > 0.0 && d < 4.0) {
    lane = LaneType::LEFT;
  } else if (d > 4.0 && d < 8.0) {
    lane = LaneType::MID;
  } else if (d > 8.0 && d < 12.0) {
    lane = LaneType::RIGHT;
  } else
    lane = LaneType::UNSPECIFIED;
  return lane;
}

LaneType Vehicle::convert_d_to_lane() const {
  return convert_d_to_lane(d);
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

double Vehicle::getTargetSpeed() { return 0.0; }