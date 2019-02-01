#include "trajectory.h"
#include "log.h"

extern  Logger log_;
// boundaries of acceptable speed of our vehicle
const double HARD_SPEED_LIMIT = 22.352;  // 50mph in m/s
const double SPEED_LIMIT = 22;
const double MIN_SPEED = 15.0;

// if the gap is less than this we consider it unsafe to turn
const double FRONT_GAP_THRESH = 25.0;
const double BACK_GAP_THRESH = 10.0;

// This is the buffers we want against the leading front vehicle
// for safety so we don't collide with the vehicle right in front of us
const double FRONT_BUFFER = FRONT_GAP_THRESH + 10.0;
const double DISTANCE_BUFFER = 5.0;
const double SPEED_BUFFER = 6.0;

inline double deg2rad(double x) { return x * M_PI / 180; }
inline double mph_to_ms(double mph) { return mph / 2.24; }



TrajectoryJMT Trajectory::JMT_init(double car_s, double car_d)
{
  TrajectoryJMT traj_jmt;
  // 50 x {s, s_dot, s_ddot}
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

Trajectory::Trajectory(Vehicle &car, const BehaviorType behavior, Map &map,
                       Config &cfg)
    : cfg_(cfg), map_(map) {
  /* 
  // get target states based on behavior s component
  double target_s =
      car.saved_state_s.p + cfg.traverseTime() * car.saved_state_s.v;
  double target_v = car.saved_state_s.v;

  // target acceleration along the load is zero
  targetState_s = {target_s, target_v, 0.0};

  // get target d component state based on behavior
  // target speed and acceleration sideways of the road are both zero
  targetState_d = {car.get_target_d(behavior), 0.0, 0.0};

 */
}

JMT Trajectory::get_jmt_s() const { return jmtPair_[0]; }

JMT Trajectory::get_jmt_d() const { return jmtPair_[1]; }

TrajectoryXY Trajectory::generateTrajectory(Vehicle &car,
                                            const BehaviorType behavior,
                                            TrajectoryXY &previous_path,
                                            Target &target) {
  TrajectoryXY trajectory;
  int numPoints = target.time / cfg_.timeIncrement();

  if (behavior == BehaviorType::KEEPLANE) {
    trajectory = generateSplineBasedTrajectory(car, target, previous_path);
  } else {
    trajectory = generateJMTTrajectory(car, behavior, numPoints);
  }

  return trajectory;
}

// TODO: Incorporate previous path into the JMT generation
TrajectoryXY Trajectory::generateJMTTrajectory(Vehicle &car,
                                               const BehaviorType behavior,
                                               int numPoints) {
  // get target states based on behavior s component
  double target_s =
      car.saved_state_s.p + cfg_.traverseTime() * car.saved_state_s.v;
  double target_v = car.saved_state_s.v;

  if (behavior == BehaviorType::KEEPLANE) {
    // If the car in front is going fast or we are very far from it anyway, go
    // as fast as we can Else let's go a notch slower than the car in front
    bool safe = (car.front_v > SPEED_LIMIT) || (car.front_gap > FRONT_BUFFER);
    target_v = safe ? SPEED_LIMIT : (car.front_v - SPEED_BUFFER);

    // But if the car in front is too slow, let's go a little faster
    target_v = target_v > MIN_SPEED ? target_v : MIN_SPEED;

    // Estimate a safe target distance based on our selected speed
    target_s = car.saved_state_s.p +
               cfg_.traverseTime() * 0.5 * (car.saved_state_s.v + target_v);
  }

  // target acceleration along the load is zero
  targetState_s = {target_s, target_v, 0.0};

  // get target d component state based on behavior
  // target speed and acceleration sideways of the road are both zero
  targetState_d = {car.get_target_d(behavior), 0.0, 0.0};

  // generate JMTs
  JMT jmt_s(car.saved_state_s, targetState_s, cfg_.traverseTime());
  JMT jmt_d(car.saved_state_d, targetState_d, cfg_.traverseTime());

  vector<double> xs;
  vector<double> ys;
  vector<double> p;

  for (int i = 0; i < numPoints; i++) {
    double s = jmt_s.get(i * cfg_.timeIncrement());
    double d = jmt_d.get(i * cfg_.timeIncrement());

    vector<double> p = map_.getXY(s, d);

    xs.push_back(p[0]);
    ys.push_back(p[1]);
  }

  XYPoints path = {xs, ys, numPoints};

  return TrajectoryXY(xs, ys);
}

// We use Spline generated trajectories only for cases where we don't have to
// switch lanes
TrajectoryXY
Trajectory::generateSplineBasedTrajectory(Vehicle &car, Target &target,
                                          TrajectoryXY &previous_path) {
  TrajectoryXY previous_path_xy = previous_path;
  int prev_size = previous_path.pts.n;

  vector<double> previous_path_x = previous_path_xy.pts.xs;
  vector<double> previous_path_y = previous_path_xy.pts.ys;

  vector<double> ptsx;
  vector<double> ptsy;

  double ref_x = car.x;
  double ref_y = car.y;
  double ref_yaw = deg2rad(car.yaw);

  if (prev_size < 2) {
    double prev_car_x = car.x - cos(car.yaw);
    double prev_car_y = car.y - sin(car.yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(car.y);
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // We use Spline generated trajectories only for cases where we don't have to
  // switch lanes
  vector<double> next_wp0 =
      map_.getXYspline(car.s + 30, car.get_target_d(BehaviorType::KEEPLANE));
  vector<double> next_wp1 =
      map_.getXYspline(car.s + 60, car.get_target_d(BehaviorType::KEEPLANE));
  vector<double> next_wp2 =
      map_.getXYspline(car.s + 90, car.get_target_d(BehaviorType::KEEPLANE));

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degrees
    // transformation to local car's coordinates (cf MPC)
    // last point of previous path at origin and its angle at zero degree

    // shift and rotation
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  tk::spline spl;
  spl.set_points(ptsx, ptsy);

  vector<double> next_x_vals;
  vector<double> next_y_vals;

  for (int i = 0; i < prev_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  // Calculate how to break up spline points so that we travel at our desired
  // reference velocity
  double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;

  /* // If the car in front is going fast or we are very far from it anyway, go as
  // fast as we can Else let's go a notch slower than the car in front
  bool safe = (car.front_v > SPEED_LIMIT) || (car.front_gap > FRONT_BUFFER);
  double target_v = safe ? SPEED_LIMIT : (car.front_v - SPEED_BUFFER);

  // But if the car in front is too slow, let's go a little faster
  target_v = target_v > MIN_SPEED ? target_v : MIN_SPEED; */

  // fill up the rest of our path planner after filing it with previous points
  // here we will output the number of points defined in the planahead config.json parameter
  for (int i = 1; i <= cfg_.planAhead() - prev_size; i++) {
    // *************************
    // Todo: change to allow for acceleration / deceleration up to a defined velocity
    // *************************
    double N = (target_dist / (cfg_.timeIncrement() *
                               target.velocity));  // divide by 2.24: mph -> m/s
    double x_point = x_add_on + target_x / N;
    double y_point = spl(x_point);

    x_add_on = x_point;

    double x_ref = x_point;  // x_ref IS NOT ref_x !!!
    double y_ref = y_point;

    // rotate back to normal after rotating it earlier
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }

  return TrajectoryXY(next_x_vals, next_y_vals);
}



TrajectoryJMT Trajectory::generateSDTrajectory(Vehicle &car,
                                               PreviousPath &previous_path,
                                               Target &target) {
  TrajectoryJMT traj_jmt;

  //for (int i = 0; i < previous_path.xy.pts.xs.size(); i++)
  //              cout << "prev x = " << previous_path.xy.pts.xs[i] << "\tprev y = " << previous_path.xy.pts.ys[i] << endl;

  TrajectoryXY previous_path_xy = previous_path.xy;
  int prev_size = previous_path.num_xy_reused;
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
  //log_.write("***** generate_trajectory_sd  *****");
  for (int i = prev_size; i < cfg_.planAhead(); i++) {
    // increase/decrease speed till target velocity is reached
    s_dot += s_ddot * cfg_.timeIncrement();

    if (s_ddot > 0) { // acceleration
      // cap at target speed if we are accelerating
      s_dot = max(min(s_dot, target.velocity), 0.0);
    }
    else
    {
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
    //if (s_ddot < 0)
    //  log_.of_ << "s = " << s  << "\ts_dot = " << s_dot << "\ts_ddot = " << s_ddot << "\td = " << d << "\tx = " << x << "\ty = " << y << endl;
    next_x_vals.push_back(point_xy[0]);
    next_y_vals.push_back(point_xy[1]);
    
  }
  //log_.write("***** ======== *****");
  traj_jmt.trajectory = TrajectoryXY(next_x_vals, next_y_vals);
  traj_jmt.path_sd = TrajectorySD(new_path_s, new_path_d);

  return traj_jmt;
}