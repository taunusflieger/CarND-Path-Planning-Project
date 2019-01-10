#include "config.h"

#include <fstream>
#include <iostream>

using json = nlohmann::json;


Config::Config() 
{
    

}

bool Config::Load(string filename) {
    std::ifstream ifs(filename);
    
    j = json::parse(ifs);
    return true;
}

int Config::w() {
    return j["viz"]["w"];
}

int Config::h() {
    return j["viz"]["h"];
}

double Config::VizTranslateX() {
    return j["viz"]["translate"]["x"];
}

double Config::VizTranslateY() {
    return j["viz"]["translate"]["y"];
}

double Config::VizScale() {
    return j["viz"]["scale"];
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


double Config::initialS() {
    return j["initial-s"];
}

