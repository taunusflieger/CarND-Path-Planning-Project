#pragma once

#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "types.h"

class JMT {

  public:
    Eigen::VectorXd c;
    JMT(const State& start, const State& end, const double t);
    double get(const double t) const;
};
