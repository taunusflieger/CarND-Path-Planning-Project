#pragma once

#include <memory>
#include <experimental/optional>
#include "json.hpp"
using namespace std;

class Config
{
public:
    Config();

    bool Load(string filename);

    int w();
    int h();
    double VizTranslateX();
    double VizTranslateY();
    double VizScale();
    double laneWidth();

    // center point of the track
    double centerX();
    double centerY();
    double trackLength();
    int numLanes();
    double sensorRangeFront();
    double sensorRangeBack();
    double targetSpeed();
    double speedLimit();
    double planAhead();

    double trajectoryWaypointDist();
    double trajectoryTrajectoryLength();
    double trajectoryTrajectoryMin();
    double trajectoryReuseNPoints();

    double speedIncrease();
    double speedTolerance();
    double collisionBuffer();

    double initialS();



private:
    static Config *_instance;
    
    nlohmann::json j;
};


