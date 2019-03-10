#include "vehicle.h"
#include "log.h"
extern Logger log_;

Vehicle::Vehicle(SensorFusionData &sf, Config &cfg) : cfg_(cfg) {
  lane = LaneType::UNSPECIFIED;
  id = sf.id;
  x = sf.x;
  y = sf.y;
  s = sf.s;
  d = sf.d;
  vx = sf.vx;
  vy = sf.vy;
  lane = convert_d_to_lane(d);
  v = sqrt(vx * vx + vy * vy);
}

void Vehicle::update_position(const double s_new, const double d_new) {
  s = s_new;
  d = d_new;
  lane = convert_d_to_lane(d);
}

void Vehicle::update_speed(const double v_new) { v = v_new; }

LaneType Vehicle::convert_d_to_lane(const double d) const {
  LaneType lane = LaneType::NONE;

  if (d > 0.0 & d < 4.0) {
    lane = LaneType::LEFT;
  } else if (d >= 4.0 & d < 8.0) {
    lane = LaneType::MID;
  } else if (d >= 8.0 & d < 12.0) {
    lane = LaneType::RIGHT;
  } else
    lane = LaneType::UNSPECIFIED;

  return lane;
}

LaneType Vehicle::convert_d_to_lane() const { return convert_d_to_lane(d); }

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

double Vehicle::convert_lane_to_d() { return convert_lane_to_d(lane); }
