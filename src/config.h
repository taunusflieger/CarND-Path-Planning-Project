#pragma once

#include "json.hpp"
#include <experimental/optional>
#include <memory>
using namespace std;

class Config {
private:
  bool loaded;

  nlohmann::json j;

public:
  Config() { loaded = false; };
  bool Load(string filename);

  double laneWidth();
  int dbgMain();

  // center point of the track
  double centerX();
  double centerY();
  double trackLength();
  int numLanes();
  int pathSizeCutOff();
  double sensorRangeFront();
  double sensorRangeBack();
  double targetSpeed();
  double speedLimit();
  double planAhead();
  double timeIncrement();

  double trajectoryWaypointDist();
  double trajectoryTrajectoryLength();
  double trajectoryTrajectoryMin();
  double trajectoryReuseNPoints();
  double traverseTime();

  double speedIncrease();
  double speedTolerance();
  double collisionBuffer();

  // Bounding box for SAT based collision check
  double carSafetyLength();
  double carSafetyWidth();

  double initialS();
};
