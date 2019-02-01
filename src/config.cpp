#include "config.h"

#include <fstream>
#include <iostream>

using json = nlohmann::json;

bool Config::Load(string filename) {
  std::ifstream ifs;

  ifs.exceptions(ifstream::badbit);

  try {
    ifs.open(filename);
    j = json::parse(ifs);
    ifs.close();
    loaded = true;

  } catch (const ifstream::failure& e) {
    cout << "Exception during reading config.json" << endl;
  }

  return loaded;
}

int Config::dbgMain() { return j["dbgMain"]; }

double Config::centerX() { return j["centerX"]; }

double Config::centerY() { return j["centerY"]; }

double Config::laneWidth() { return j["laneWidth"]; }

double Config::sensorRangeFront() { return j["sensor"]["range-front"]; }

double Config::sensorRangeBack() { return j["sensor"]["range-back"]; }

int Config::numLanes() { return j["numLanes"]; }

double Config::trackLength() { return j["trackLength"]; }

double Config::targetSpeed() { return j["targetSpeed"]; }

double Config::acceleration() { return j["acceleration"]; }

double Config::accelerationLimit(){ return j["accelerationLimit"]; }

double Config::speedLimit() { return j["speedLimit"]; }

double Config::planAhead() { return j["planAhead"]; }

double Config::collisionBuffer() { return j["collisionBuffer"]; }

double Config::trajectoryWaypointDist() {
  return j["trajectory"]["waypoint-dist"];
}

double Config::trajectoryTrajectoryLength() {
  return j["trajectory"]["trajectory-length"];
}

double Config::trajectoryTrajectoryMin() {
  return j["trajectory"]["trajectory-min"];
}

double Config::trajectoryReuseNPoints() {
  return j["trajectory"]["reuse-n-points"];
}

double Config::carSafetyLength() { return j["car"]["safetyLength"]; }

double Config::carSafetyWidth() { return j["car"]["safetyWidth"]; }

double Config::speedIncrease() { return j["speedIncrease"]; }

double Config::speedTolerance() { return j["speedTolerance"]; }

int Config::pathSizeCutOff() { return j["pathSizeCutOff"]; }

int Config::prevPathReuse() { return j["prevPathReuse"]; }

double Config::timeIncrement() { return j["timeIncrement"]; }

double Config::traverseTime() { return j["traverseTime"]; }
