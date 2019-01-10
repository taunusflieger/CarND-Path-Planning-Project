#ifndef MAP_H
#define MAP_H
#include <vector>
#include "json.hpp"
#include "spline.h"

using namespace std;
using namespace tk;

// The max s value before wrapping around the track back to 0
const double MAX_S = 6945.554;

// center point of the track
const double PARAM_CENTER_X = 1000;
const double PARAM_CENTER_Y = 2000;






class Map {
  private:
    struct PointIdx {
      double value;
      int index;
    };

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
    Map() {};
    ~Map() {};

    void LoadData(string& filename);

    vector<double> getXY(double s, double d);
    double distance(double x1, double y1, double x2, double y2);
    int ClosestWaypoint(double x, double y);
    int ClosestWaypoint2(double x, double y);
    void TestClosestWaypoint();
    int NextWaypoint(double x, double y, double theta);
    vector<double> getFrenet(double x, double y, double theta);
    void plot(void);
};

#endif
