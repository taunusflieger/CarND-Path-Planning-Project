#pragma once

#include "config.h"
#include "types.h"

class Cost {
public:
  Cost(Config &cfg);
  bool checkCollision(double s0, double d0, double theta0, double s1, double d1,
                      double theta1);

private:
  Config &cfg_;


};
