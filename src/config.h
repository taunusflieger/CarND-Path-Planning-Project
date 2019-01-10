#ifndef CONFIG_H
#define CONFIG_H

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


#endif // CONFIG_H