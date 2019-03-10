#pragma once

#include "config.h"
#include "types.h"
#include "prediction.h"

class Cost {
public:
 Cost(Config &cfg);
 double getCost(TrajectoryCandidate& tc, Prediction &predictions, std::vector<Vehicle> &otherCars, Vehicle &egoCar);

private:
 Config &cfg_;

};
