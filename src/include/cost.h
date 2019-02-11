#pragma once

#include "config.h"
#include "types.h"
#include "prediction.h"

class Cost {
public:
 Cost(Config &cfg);
 double getCost(TrajectoryCandidates& tc, Prediction &predictions);

private:
 Config &cfg_;

 bool checkCollision(double s0, double d0, double theta0, double s1, double d1,
                     double theta1);
};
