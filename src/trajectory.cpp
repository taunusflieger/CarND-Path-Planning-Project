#include "trajectory.h"
#include "Eigen-3.3/Eigen/Dense"
#include "log.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

extern Logger log_;

inline double deg2rad(double x) { return x * M_PI / 180; }
inline double mph_to_ms(double mph) { return mph / 2.24; }

// d coord for center lane
double Trajectory::getDPosition(LaneType lane) {
  double dlane = static_cast<double>(lane);
  double dcenter = (dlane + 0.5) * cfg_.laneWidth();
  if (dcenter >= 10) {
    dcenter = 9.8;  
  }
  return dcenter;
}

TrajectoryJMT Trajectory::JMT_init(double car_s, double car_d) {
  TrajectoryJMT traj_jmt;

  vector<PointCmp> store_path_s(cfg_.planAhead(), PointCmp(0, 0, 0));
  vector<PointCmp> store_path_d(cfg_.planAhead(), PointCmp(0, 0, 0));

  for (int i = 0; i < cfg_.planAhead(); i++) {
    store_path_s[i] = PointCmp(car_s, 0, 0);
    store_path_d[i] = PointCmp(car_d, 0, 0);
  }

  traj_jmt.path_sd.path_s = store_path_s;
  traj_jmt.path_sd.path_d = store_path_d;

  return traj_jmt;
}

// c: coefficients of polynom
double Trajectory::polyeval(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 0; i < c.size(); i++) {
    res += c[i] * pow(t, i);
  }
  return res;
}

// 1st derivative of a polynom
double Trajectory::polyeval_dot(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 1; i < c.size(); ++i) {
    res += i * c[i] * pow(t, i - 1);
  }
  return res;
}

// 2nd derivative of a polynom
double Trajectory::polyeval_ddot(vector<double> c, double t) {
  double res = 0.0;
  for (size_t i = 2; i < c.size(); ++i) {
    res += i * (i - 1) * c[i] * pow(t, i - 2);
  }
  return res;
}

Trajectory::Trajectory(Vehicle &car, const BehaviorType behavior, Map &map,
                       Config &cfg)
    : cfg_(cfg), map_(map) {
}

TrajectoryJMT Trajectory::generate_trajectory_jmt(Target target, Map &map, PreviousPath const &previous_path) {
  TrajectoryJMT traj_jmt;
  double max_speed_inc = cfg_.timeIncrement() * cfg_.acceleration();
  TrajectoryXY previous_path_xy = previous_path.xy;
  int prev_size = previous_path.num_xy_reused;
  TrajectorySD prev_path_sd = previous_path.sd;
  vector<double> previous_path_x = previous_path_xy.pts.xs;
  vector<double> previous_path_y = previous_path_xy.pts.ys;
  vector<PointCmp> prev_path_s = prev_path_sd.path_s;
  vector<PointCmp> prev_path_d = prev_path_sd.path_d;

  vector<PointCmp> new_path_s(cfg_.planAhead(), PointCmp(0, 0, 0));
  vector<PointCmp> new_path_d(cfg_.planAhead(), PointCmp(0, 0, 0));

  int last_point;
  if (cfg_.prevPathReuse() < cfg_.planAhead()) {
    last_point = cfg_.planAhead() - previous_path_x.size() + prev_size - 1;
  } else {
    last_point = cfg_.planAhead() - 1;
  }

  double T = target.time; 

  double si, si_dot = 0, si_ddot;
  double di, di_dot, di_ddot;

  si = prev_path_s[last_point].v;
  si_dot = prev_path_s[last_point].v_dot;
  si_ddot = prev_path_s[last_point].v_ddot;

  di = prev_path_d[last_point].v;
  di_dot = prev_path_d[last_point].v_dot;
  di_ddot = prev_path_d[last_point].v_ddot;

  double sf, sf_dot, sf_ddot;
  double df, df_dot, df_ddot;

  if (target.velocity <= 10) {
    df_ddot = 0;
    df_dot = 0;
    df = di;

    sf_ddot = 0;
    sf_dot = mph_to_ms(target.velocity);

    sf_dot = min(sf_dot, si_dot + 10 * max_speed_inc);
    sf_dot = max(sf_dot, si_dot - 10 * max_speed_inc);

    sf = si + 2 * sf_dot * T;
  } else {
    df_ddot = 0;
    df_dot = 0;
    df = getDPosition(target.lane);

    sf_ddot = 0;
    sf_dot = target.velocity;

    // we use JMT for lane changes only
    // no need to try to reach amx speed during lane changes
    sf_dot = min(sf_dot, 0.9 * cfg_.speedLimit());
    sf_dot = min(sf_dot, si_dot + 10 * max_speed_inc);
    sf_dot = max(sf_dot, si_dot - 10 * max_speed_inc);

    sf = si + sf_dot * T;
  }

  /////////////////////////////////////////////////////////////
  State start_s, end_s;
  State start_d, end_d;
  start_s.p = si;
  start_s.v = si_dot;
  start_s.a = si_ddot;

  end_s.p = sf;
  end_s.v = sf_dot;
  end_s.a = 0;

  start_d.p = di;
  start_d.v = di_dot;
  start_d.a = di_ddot;

  end_d.p = df;
  end_d.v = df_dot;
  end_d.a = df_ddot;

  JMT jmt_s(start_s, end_s, T);
  JMT jmt_d(start_d, end_d, T);

  vector<double> poly_s(jmt_s.c.data(), jmt_s.c.data() + jmt_s.c.rows() * jmt_s.c.cols());
  vector<double> poly_d(jmt_d.c.data(), jmt_d.c.data() + jmt_d.c.rows() * jmt_d.c.cols());

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  for (int i = 0; i < prev_size; i++) {
    new_path_s[i] = prev_path_s[cfg_.planAhead() - previous_path_x.size() + i];
    new_path_d[i] = prev_path_d[cfg_.planAhead() - previous_path_x.size() + i];

    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double t = cfg_.timeIncrement();
  for (int i = prev_size; i < cfg_.planAhead(); i++) {
    double s = polyeval(poly_s, t);
    double s_dot = polyeval_dot(poly_s, t);
    double s_ddot = polyeval_ddot(poly_s, t);

    double d = polyeval(poly_d, t);
    double d_dot = polyeval_dot(poly_d, t);
    double d_ddot = polyeval_ddot(poly_d, t);

    new_path_s[i] = PointCmp(s, s_dot, s_ddot);
    new_path_d[i] = PointCmp(d, d_dot, d_ddot);

    vector<double> point_xy = map.getXYspline(s, d);

    next_x_vals.push_back(point_xy[0]);
    next_y_vals.push_back(point_xy[1]);

    t += cfg_.timeIncrement();
  }

  traj_jmt.trajectory = TrajectoryXY(next_x_vals, next_y_vals);
  traj_jmt.path_sd = TrajectorySD(new_path_s, new_path_d);
  return traj_jmt;
}

TrajectoryJMT Trajectory::generateSDTrajectory(Vehicle &car,
                                               PreviousPath &previous_path,
                                               Target &target) {
  TrajectoryJMT traj_jmt;
  log_.write("***** BEGIN generate_trajectory_sd  *****");
  log_.of_ << "previous_path_xy size = " << previous_path.xy.pts.xs.size() << endl;
  TrajectoryXY previous_path_xy = previous_path.xy;
  int prev_size = previous_path.xy.pts.xs.size();  // previous_path.num_xy_reused;
  TrajectorySD prev_path_sd = previous_path.sd;

  vector<double> previous_path_x = previous_path_xy.pts.xs;
  vector<double> previous_path_y = previous_path_xy.pts.ys;
  vector<PointCmp> prev_path_s = prev_path_sd.path_s;
  vector<PointCmp> prev_path_d = prev_path_sd.path_d;

  vector<PointCmp> new_path_s(cfg_.planAhead(), PointCmp(0, 0, 0));
  vector<PointCmp> new_path_d(cfg_.planAhead(), PointCmp(0, 0, 0));

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  double target_velocity_ms = target.velocity;

  double s, s_dot, s_ddot;
  double d, d_dot, d_ddot;
  if (prev_size > 3)
    prev_size = 3;
  if (prev_size > 0) {
    for (int i = 0; i < prev_size; i++) {
      int len = previous_path_x.size();
      int pa = cfg_.planAhead();
      int b = prev_path_s.size();
      new_path_s[i] = prev_path_s[cfg_.planAhead() - previous_path_x.size() + i];
      new_path_d[i] = prev_path_d[cfg_.planAhead() - previous_path_x.size() + i];
      next_x_vals.push_back(previous_path_x[i]);
      next_y_vals.push_back(previous_path_y[i]);
    }

    // initial conditions for new (s,d) trajectory
    s = new_path_s[prev_size - 1].v;
    s_dot = new_path_s[prev_size - 1].v_dot;
    d = new_path_d[prev_size - 1].v, d_dot = 0, d_ddot = 0;
  } else {
    s = car.s, s_dot = car.v;
    d = car.d, d_dot = 0, d_ddot = 0;
  }

  s_ddot = target.acceleration;

  double prev_s_dot = s_dot;

  for (int i = prev_size; i < cfg_.planAhead(); i++) {
    // increase/decrease speed till target velocity is reached
    s_dot += s_ddot * cfg_.timeIncrement();

    if (s_ddot > 0) {  // acceleration
      // cap at target speed if we are accelerating
      s_dot = max(min(s_dot, target.velocity), 0.0);
    } else {
      // cap at 0 if we are decelerate
      s_dot = max(s_dot, 0.0);
    }

    s += s_dot * cfg_.timeIncrement();

    prev_s_dot = s_dot;

    new_path_s[i] = PointCmp(s, s_dot, s_ddot);
    new_path_d[i] = PointCmp(d, d_dot, d_ddot);

    vector<double> point_xy = map_.getXYspline(s, d);
    double x = point_xy[0];
    double y = point_xy[1];
    next_x_vals.push_back(point_xy[0]);
    next_y_vals.push_back(point_xy[1]);
  }

  log_.write("***** END generate_trajectory_sd  *****");
  traj_jmt.trajectory = TrajectoryXY(next_x_vals, next_y_vals);
  traj_jmt.path_sd = TrajectorySD(new_path_s, new_path_d);
 
  return traj_jmt;
}