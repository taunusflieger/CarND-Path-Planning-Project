#pragma once

#include <vector>
#include "config.h"
#include "jmt.h"
#include "json.hpp"
#include "spline.h"

using namespace std;
using namespace tk;

struct Point {
  double x;
  double y;
};

class Map {
 private:
  struct PointIdx {
    double value;
    int index;
  };

  Config &cfg_;

  spline spline_x_;
  spline spline_y_;
  spline spline_dx_;
  spline spline_dy_;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x_;
  vector<double> map_waypoints_y_;
  vector<double> map_waypoints_s_;
  vector<double> map_waypoints_dx_;
  vector<double> map_waypoints_dy_;
  vector<double> map_s_;

  // high resultion map 1 point per meter
  vector<double> high_res_map_waypoints_x_;
  vector<double> high_res_map_waypoints_y_;
  vector<double> high_res_map_waypoints_dx_;
  vector<double> high_res_map_waypoints_dy_;
  vector<double> high_res_map_waypoints_s_;

  vector<PointIdx> high_res_map_waypoints_sorted_x_;
  vector<PointIdx> high_res_map_waypoints_sorted_y_;

 public:
  Map(Config &cfg) : cfg_(cfg){};
  ~Map(){};

  void LoadData(string &filename);

  vector<double> getXY(double s, double d);
  vector<double> getXY2(double s, double d);
  vector<double> getXYspline(double s, double d);
  double distance(double x1, double y1, double x2, double y2);
  int ClosestWaypoint(double x, double y);
  int ClosestWaypoint2(double x, double y);
  int NextWaypoint(double x, double y, double theta);
  XYPoints makePath(JMT jmt_s, JMT jmt_d, const double t, const int n);
  vector<double> getFrenet(double x, double y, double theta);
  double deg2rad(double x) { return x * M_PI / 180; };
};
