#pragma once

#include <vector>
#include "config.h"
#include "jmt.h"
#include "json.hpp"
#include "spline.h"


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

  tk::spline spline_x_;
  tk::spline spline_y_;
  tk::spline spline_dx_;
  tk::spline spline_dy_;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  std::vector<double> map_waypoints_x_;
  std::vector<double> map_waypoints_y_;
  std::vector<double> map_waypoints_s_;
  std::vector<double> map_waypoints_dx_;
  std::vector<double> map_waypoints_dy_;
  std::vector<double> map_s_;

  // high resultion map 1 point per meter
  std::vector<double> high_res_map_waypoints_x_;
  std::vector<double> high_res_map_waypoints_y_;
  std::vector<double> high_res_map_waypoints_dx_;
  std::vector<double> high_res_map_waypoints_dy_;
  std::vector<double> high_res_map_waypoints_s_;

  std::vector<PointIdx> high_res_map_waypoints_sorted_x_;
  std::vector<PointIdx> high_res_map_waypoints_sorted_y_;

 public:
  Map(Config &cfg) : cfg_(cfg){};
  ~Map(){};

  void LoadData(string &filename);

  std::vector<double> getXY(double s, double d);
  std::vector<double> getXY2(double s, double d);
  std::vector<double> getXYspline(double s, double d);
  double distance(double x1, double y1, double x2, double y2);
  int ClosestWaypoint(double x, double y);
  int ClosestWaypoint2(double x, double y);
  int NextWaypoint(double x, double y, double theta);
  XYPoints makePath(JMT jmt_s, JMT jmt_d, const double t, const int n);
  std::vector<double> getFrenet(double x, double y, double theta);
  double deg2rad(double x) { return x * M_PI / 180; };
};
