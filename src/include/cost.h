#pragma once

#include "config.h"
#include "types.h"
#include "prediction.h"

class Cost {
public:
 Cost(Config &cfg);
 double getCost(TrajectoryCandidate& tc, Prediction &predictions);

private:
 Config &cfg_;

 
};
