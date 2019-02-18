

#include <math.h>
#include <iostream>
#include <vector>


#include "cost.h"


using namespace std;


Cost::Cost(Config &cfg) : cfg_(cfg) {}


double Cost::getCost(TrajectoryCandidate& tc, Prediction &predictions) {
  double cost = 0;

  if (predictions.IsTrajectoryCollisionFree(tc))
    cost += 10000;

  return cost;
}
