#include "map.h"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#define WITHOUT_NUMPY
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

// For converting back and forth between radians and degrees.
inline double deg2rad(double x) { return x * M_PI / 180; }
inline double rad2deg(double x) { return x * 180 / M_PI; }

void Map::LoadData(string &filename) {
  ifstream in_map(filename.c_str(), ifstream::in);

  string line;

  cout << "Speed Limit: " << cfg_.speedLimit() << endl;
  double prev_x, prev_y;
  double max_dist = 0, min_dist = MAXFLOAT;
  double total_dist = 0;
  int line_cnt = 0;
  while (getline(in_map, line)) {
    istringstream iss(line);
    double x, y;
    float s, d_x, d_y;

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x_.push_back(x);
    map_waypoints_y_.push_back(y);
    map_waypoints_s_.push_back(s);
    map_waypoints_dx_.push_back(d_x);
    map_waypoints_dy_.push_back(d_y);
    if (line_cnt > 0) {
      double dist = distance(x, y, prev_x, prev_y);
      if (dist > max_dist)
        max_dist = dist;
      if (dist < min_dist)
        min_dist = dist;
      total_dist += dist;
    }
    prev_x = x;
    prev_y = y;
    line_cnt++;
  }
  in_map.close();
  assert(map_waypoints_x_.size() &&
         "map not loaded, probably path include is missing");

  cout << "Original Map intra point distance min = " << min_dist << "m" << endl;
  cout << "                                  max = " << max_dist << "m" << endl;
  cout << "                                  avg = "
       << total_dist / (double)line_cnt << "m" << endl;

  // to get a good spline approximation on last segment wrapping around
  map_waypoints_x_.push_back(map_waypoints_x_.front());
  map_waypoints_y_.push_back(map_waypoints_y_.front());
  map_waypoints_s_.push_back(cfg_.trackLength());
  map_waypoints_dx_.push_back(map_waypoints_dx_.front());
  map_waypoints_dy_.push_back(map_waypoints_dy_.front());

  // Interpolate to increase resultion to 1 pt per meter
  spline_x_.set_points(map_waypoints_s_, map_waypoints_x_);
  spline_y_.set_points(map_waypoints_s_, map_waypoints_y_);
  spline_dx_.set_points(map_waypoints_s_, map_waypoints_dx_);
  spline_dy_.set_points(map_waypoints_s_, map_waypoints_dy_);

  // remove duplicated wrapped point
  map_waypoints_x_.pop_back();
  map_waypoints_y_.pop_back();
  map_waypoints_s_.pop_back();
  map_waypoints_dx_.pop_back();
  map_waypoints_dy_.pop_back();

  double frenet_s = 0.0;
  map_s_.push_back(0.0);
  for (size_t i = 1; i < map_waypoints_x_.size(); i++) {
    frenet_s += distance(map_waypoints_x_[i], map_waypoints_y_[i],
                         map_waypoints_x_[i - 1], map_waypoints_y_[i - 1]);
    map_s_.push_back(frenet_s);
  }

  // Create new high resultion map using the spline functions
  total_dist = 0;
  max_dist = 0;
  min_dist = MAXFLOAT;
  prev_x = spline_x_(0);
  prev_y = spline_y_(0);
  int idx = 0;
  for (double s = 0; s <= floor(cfg_.trackLength()); s++) {
    double x = spline_x_(s);
    double y = spline_y_(s);
    double dist = distance(x, y, prev_x, prev_y);

    if (dist > max_dist)
      max_dist = dist;
    if (dist < min_dist)
      min_dist = dist;
    total_dist += dist;

    prev_x = x;
    prev_y = y;

    high_res_map_waypoints_x_.push_back(x);
    high_res_map_waypoints_y_.push_back(y);
    high_res_map_waypoints_dx_.push_back(spline_dx_(s));
    high_res_map_waypoints_dy_.push_back(spline_dy_(s));
    high_res_map_waypoints_s_.push_back(s);

    // cout << x << "\t" << idx << endl;
    idx++;
  }

  cout << "High Res Map intra point distance min = " << min_dist << "m" << endl;
  cout << "                                  max = " << max_dist << "m" << endl;
  cout << "                                  avg = " << total_dist / (double)idx
       << "m" << endl;
  cout << "Number of points = " << idx - 1 << endl;

  /* for (auto p : high_res_map_waypoints_sorted_x_) {
  cout << p.value << " " << p.index << endl;
}  */
}

double Map::distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int Map::ClosestWaypoint2(double x, double y) {
  const int window_size = 60;
  double closestDist = 100000; // large number
  int closestWaypoint = 0;

  // Coarse search through the map to identify area for
  // detailed search
  int size = high_res_map_waypoints_x_.size();
  int steps = size / window_size;

  for (int i = 0; i < size; i += steps) {
    double dist = distance(x, y, high_res_map_waypoints_x_[i],
                           high_res_map_waypoints_y_[i]);
    if (dist < closestDist) {
      closestDist = dist;
      closestWaypoint = i;
    }
  }

  // Search within the identified segment for the
  // optimal matching point
  closestDist = 100000;
  int start_idx = closestWaypoint - window_size;
  int end_idx = closestWaypoint + window_size;

  // Ensure that we are not searching out side
  // of the boundaries of the vector
  start_idx = (start_idx < 0 ? size - abs(start_idx) : start_idx);
  end_idx = (end_idx > size ? end_idx - size : end_idx);
  for (int i = start_idx; i != end_idx;) {
    double dist = distance(x, y, high_res_map_waypoints_x_[i],
                           high_res_map_waypoints_y_[i]);
    if (dist < closestDist) {
      closestDist = dist;
      closestWaypoint = i;
    }

    if (++i == size)
      i = 0;
  }
  return closestWaypoint;
}

int Map::ClosestWaypoint(double x, double y) {
  double closestLen = 100000; // large number
  int closestWaypoint = 0;

  for (int i = 0; i < high_res_map_waypoints_x_.size(); i++) {
    double map_x = high_res_map_waypoints_x_[i];
    double map_y = high_res_map_waypoints_y_[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}

XYPoints Map::makePath(JMT jmt_s, JMT jmt_d, const double t, const int n) {

  vector<double> xs;
  vector<double> ys;
  vector<double> p;

  for (int i = 0; i < n; i++) {

    double s = jmt_s.get(i * t);
    double d = jmt_d.get(i * t);

    vector<double> p = getXY(s, d);

    xs.push_back(p[0]);
    ys.push_back(p[1]);
  }

  XYPoints path = {xs, ys, n};

  return path;
}

int Map::NextWaypoint(double x, double y, double theta) {

  int closestWaypoint = ClosestWaypoint2(x, y);

  double map_x = high_res_map_waypoints_x_[closestWaypoint];
  double map_y = high_res_map_waypoints_y_[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = min(2 * M_PI - angle, angle);

  if (angle > M_PI / 4) {
    if (++closestWaypoint == high_res_map_waypoints_x_.size())
      closestWaypoint = 0;
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta) {
  int next_wp = NextWaypoint(x, y, theta);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = map_waypoints_x_.size() - 1;
  }

  double n_x =
      high_res_map_waypoints_x_[next_wp] - high_res_map_waypoints_x_[prev_wp];
  double n_y =
      high_res_map_waypoints_y_[next_wp] - high_res_map_waypoints_y_[prev_wp];
  double x_x = x - high_res_map_waypoints_x_[prev_wp];
  double x_y = y - high_res_map_waypoints_y_[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point

  double center_x = cfg_.centerX() - high_res_map_waypoints_x_[prev_wp];
  double center_y = cfg_.centerY() - high_res_map_waypoints_y_[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(
        high_res_map_waypoints_x_[i], high_res_map_waypoints_y_[i],
        high_res_map_waypoints_x_[i + 1], high_res_map_waypoints_y_[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  assert(frenet_d >= 0);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY2(double s, double d) {
  vector<double> &maps_s = map_waypoints_s_;
  vector<double> &maps_x = map_waypoints_x_;
  vector<double> &maps_y = map_waypoints_y_;

  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading =
      atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - M_PI / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d) {
  int prev_wp = -1;

  while (s > high_res_map_waypoints_s_[prev_wp + 1] &&
         (prev_wp < (int)(high_res_map_waypoints_s_.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % high_res_map_waypoints_x_.size();

  double heading = atan2(
      (high_res_map_waypoints_y_[wp2] - high_res_map_waypoints_y_[prev_wp]),
      (high_res_map_waypoints_x_[wp2] - high_res_map_waypoints_x_[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - high_res_map_waypoints_s_[prev_wp]);

  double seg_x = high_res_map_waypoints_x_[prev_wp] + seg_s * cos(heading);
  double seg_y = high_res_map_waypoints_y_[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - M_PI / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

vector<double> Map::getXYspline(double s, double d) {
  s = fmod(s, cfg_.trackLength());
  double x = spline_x_(s) + d * spline_dx_(s);
  double y = spline_y_(s) + d * spline_dy_(s);
  return {x, y};
}

void Map::plot(void) {
  plt::title("Map");
  plt::plot(map_waypoints_x_, map_waypoints_y_, "b.");

  vector<double> car_x = {1, 770.0906};
  vector<double> car_y = {1, 1129.872};
  plt::plot(car_x, car_y, "gD");
  plt::show();
}

void Map::TestClosestWaypoint() {
  clock_t start, end;
  double cpu_time_used;
  int error_cnt = 0;
  for (int i = 0; i < high_res_map_waypoints_x_.size(); i++) {
    double map_x = high_res_map_waypoints_x_[i] + 0.9;
    double map_y = high_res_map_waypoints_y_[i] - 1.1;

    int index1, index2;
    index1 = ClosestWaypoint(map_x, map_y);
    index2 = ClosestWaypoint2(map_x, map_y);

    if (index1 != index2) {
      cout << "Not matching index1=" << index1 << "\t index2=" << index2
           << endl;
      double dist1 = distance(high_res_map_waypoints_x_[index1],
                              high_res_map_waypoints_y_[index1], map_x, map_y);
      double dist2 = distance(high_res_map_waypoints_x_[index2],
                              high_res_map_waypoints_y_[index2], map_x, map_y);
      cout << "dist1 = " << dist1 << "\t dist2 = " << dist2 << endl;
      error_cnt++;
    }
  }
  cout << "Total error = " << error_cnt << endl;

  start = clock();
  for (int i = 0; i < high_res_map_waypoints_x_.size(); i++) {
    double map_x = high_res_map_waypoints_x_[i] + 0.5;
    double map_y = high_res_map_waypoints_y_[i] - 0.5;

    int index1, index2;
    index1 = ClosestWaypoint(map_x, map_y);
  }
  end = clock();
  cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;

  cout << "Time for ClosestWaypoint() " << cpu_time_used << " sec" << endl;

  start = clock();
  for (int i = 0; i < high_res_map_waypoints_x_.size(); i++) {
    double map_x = high_res_map_waypoints_x_[i] + 0.5;
    double map_y = high_res_map_waypoints_y_[i] - 0.5;

    int index1, index2;
    index1 = ClosestWaypoint2(map_x, map_y);
  }
  end = clock();
  cpu_time_used = ((double)(end - start)) / CLOCKS_PER_SEC;

  cout << "Time for ClosestWaypoint2() " << cpu_time_used << " sec" << endl;
}
