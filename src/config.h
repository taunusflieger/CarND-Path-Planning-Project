#pragma once

#include <memory>
#include <experimental/optional>
#include "json.hpp"
using namespace std;

class Config
{
    private:
       

        bool loaded;

        nlohmann::json j;   
    public:
        Config() { loaded = false;};
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

        double initialS(); 
};
