#include "config.h"

#include <fstream>
#include <iostream>

using json = nlohmann::json;


bool Config::Load(string filename) {
    std::ifstream ifs(filename);
    
    j = json::parse(ifs);
    loaded = true;
    return true;
}



int Config::dbgMain() {
    return j["dbgMain"];
}


double Config::centerX() {
    return j["centerX"];
}

double Config::centerY() {
    return j["centerY"];
}

double Config::laneWidth() {
    return j["laneWidth"];
}

double Config::sensorRangeFront() {
    return j["sensor"]["range-front"];
}

double Config::sensorRangeBack() {
    return j["sensor"]["range-back"];
}

int Config::numLanes() {
    return j["numLanes"];
}

double Config::trackLength() {
    return j["trackLength"];
}

double Config::targetSpeed() {
    return j["targetSpeed"];
}

double Config::speedLimit() {
    return j["speedLimit"];
}


double Config::planAhead() {
    return j["planAhead"];
}

double Config::collisionBuffer() {
    return j["collisionBuffer"];
}

double Config::trajectoryWaypointDist() {
    return j["trajectory"]["waypoint-dist"];
}

double Config::trajectoryTrajectoryLength(){
    return j["trajectory"]["trajectory-length"];
}

double Config::trajectoryTrajectoryMin(){
    return j["trajectory"]["trajectory-min"];
}

double Config::trajectoryReuseNPoints(){
    return j["trajectory"]["reuse-n-points"];
}

double Config::speedIncrease()  {
    return j["speedIncrease"];
}

double Config::speedTolerance()  {
    return j["speedTolerance"];
}

int Config::pathSizeCutOff() {
    return j["pathSizeCutOff"];
}

double Config::timeIncrement()  {
    return j["timeIncrement"];
}

double Config::traverseTime() {
    return j["traverseTime"];
}
