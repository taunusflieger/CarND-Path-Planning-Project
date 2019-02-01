// clang-format off
#include <sys/time.h>
#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "config.h"
#include "map.h"
#include "vehicle.h"
#include "types.h"
#include "trajectory.h"
#include "prediction.h"
#include "behavior.h"
#include "log.h"
// clang-format on

using namespace std;

Logger log_("/tmp/path_planning_mz.log");

string hasData(string s);

// for convenience
using json = nlohmann::json;

// Waypoint map to read from
string map_file_ = "data/highway_map.csv";

int main(int argc, char **argv) {
  uWS::Hub h;
  bool just_starting = true;

  // Holds all the data from the config file
  Config cfg;

  Map map(cfg);
  TrajectorySD prev_path_sd;

  cfg.Load("config.json");
  cout << cfg.numLanes() << endl;
  cout << "Speed Limit: " << cfg.speedLimit() << endl;
  cout << "Track length: " << cfg.trackLength() << endl;
  cout << "Loading map...";
  map.LoadData(map_file_);
  cout << "Map loaded" << endl;

  // map.TestClosestWaypoint();

  // map.plot();

  h.onMessage([&cfg, &map, &just_starting, &prev_path_sd](uWS::WebSocket<uWS::SERVER> ws,
                                           char *data, size_t length,
                                           uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    static double v_old = 0;
    static double call_cnt = 0;
    static long t_oldmsg = 0;

    // keep track of previous s and d paths: to initialize for continuity the
    // new trajectory we do the same for x,y path, but for this we get the
    // prev_path from the simulator
   

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      struct timespec tstart = {0, 0}, tend = {0, 0};
      clock_gettime(CLOCK_MONOTONIC, &tstart);

      // Log time at start of processing received data
      long t_msg = (long)chrono::time_point_cast<std::chrono::milliseconds>(
                       chrono::high_resolution_clock::now())
                       .time_since_epoch()
                       .count();

      // delta t between tow calls should be around 20ms. For the first call we
      // set it fix to 20ms
      long dt = (t_msg - t_oldmsg < 0
                     ? 20
                     : t_msg - t_oldmsg); // time between two calls (ms)
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        call_cnt++;
        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Ego car's localization Data
          Vehicle egoCar(100, cfg);
          egoCar.update_position(j[1]["s"], j[1]["d"]);
          egoCar.updatePositionXY(j[1]["x"], j[1]["y"]);
          egoCar.updateYaw(j[1]["yaw"]);
          egoCar.update_speed(((double)j[1]["speed"]) / 2.237); // mph => m/s
          egoCar.specify_adjacent_lanes();
          

          cout << "SPEEDOMETER: car.speed=" << egoCar.v << endl;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];

          // Our default is to just give back our previous plan
          int n = previous_path_x.size();
          TrajectoryXY prev_path_xy(previous_path_x, previous_path_y);

          /////////////////////////////////////////
          // Sensor Fusion
          /////////////////////////////////////////

          // A 2d vector of cars and then that car's [car's unique ID,
          // car's x position in map coordinates, car's y position in map
          // coordinates, car's x velocity in m/s, car's y velocity in m/s,
          // car's s position in frenet coordinates, car's d position
          // in frenet coordinates.
          vector<vector<double>> raw_sensor_fusion = j[1]["sensor_fusion"];

          t_oldmsg = t_msg;

          /////////////////////////////////////////
          // Localization
          /////////////////////////////////////////

          /////////////////////////////////////////
          // Trajectory
          /////////////////////////////////////////

          /////////////////////////////////////////
          // Behavior Planning
          /////////////////////////////////////////


          // Our previous plan is about to run out, so append to it
          // Make a list of all relevant information about other cars
          vector<Vehicle> otherCars;

          for (int i = 0; i < raw_sensor_fusion.size(); i++) {
            // sensor fusion data structure:
            // car's unique ID, car's x position in map coordinates, car's y
            // position in map coordinates, car's x velocity in m/s, car's y
            // velocity in m/s, car's s position in frenet coordinates, car's
            // d position in frenet coordinates.
            SensorFusionData sf(
                raw_sensor_fusion[i][0], raw_sensor_fusion[i][1],
                raw_sensor_fusion[i][2], raw_sensor_fusion[i][3],
                raw_sensor_fusion[i][4], raw_sensor_fusion[i][5],
                raw_sensor_fusion[i][6]);

            Vehicle car(sf, cfg);
            otherCars.emplace_back(car);
          }

          int prev_size = prev_path_xy.pts.xs.size();
          log_.of_ << "prev_size=" << prev_size << " car.x=" << egoCar.x << " car.y=" << egoCar.y << " car.s=" << 
                  egoCar.s << " car.d=" << egoCar.d << " car.speed=" << egoCar.v  << endl;

          vector<double> frenet_car = map.getFrenet(egoCar.x, egoCar.y, map.deg2rad(egoCar.yaw));
          egoCar.s = frenet_car[0];
          egoCar.d = frenet_car[1];

          if (just_starting) {
            Trajectory trajectory(cfg, map);
            TrajectoryJMT traj_jmt = trajectory.JMT_init(egoCar.s, egoCar.d);
/* 
            log_.write("**** Initial trajectory *****");
            for (int i = 0; i < traj_jmt.path_sd.path_s.size(); i++)
              log_.of_ << "s = " << traj_jmt.path_sd.path_s[i].v << "\td = " << traj_jmt.path_sd.path_d[i].v << endl;
            log_.write("***** Initial trajectory - created *****");
          */   
            prev_path_sd = traj_jmt.path_sd;
            just_starting = false;
          }

          /////////////////////////////////////////
          // Prediction
          /////////////////////////////////////////
          // -- prev_size: close to 100 msec when possible -not lower bcz of simulator latency- for trajectory (re)generation ---
          // points _before_ prev_size are kept from previous generated trajectory
          // points _after_  prev_size will be re-generated
          PreviousPath previous_path = PreviousPath(prev_path_xy, prev_path_sd, min(prev_path_xy.pts.n, cfg.prevPathReuse()));
          
         /*  log_.write("***** Previous path  *****");
          for (int i = 0; i < previous_path.sd.path_d.size(); i++) {
            log_.of_ << "s = " << previous_path.sd.path_s[i].v << "\td = " << previous_path.sd.path_d[i].v;
            if (i < previous_path.xy.pts.xs.size()) 
              log_.of_ << "\tx = " << previous_path.xy.pts.xs[i] << "\ty = " << previous_path.xy.pts.ys[i] << endl;
            else
              log_.of_ << endl;
           }
           log_.write("***** *************  *****");
 */
           Prediction predictions(otherCars, egoCar, cfg.planAhead(), cfg);

           Behavior behavior(otherCars, egoCar, predictions, previous_path, map, cfg);

           // Trajectory trajectory(egoCar, BehaviorType::KEEPLANE, map, cfg);

           // Update saved state of our car (THIS IS IMPORTANT) with the latest
           // generated target states, this is to be used as the starting state
           // when generating a trajectory next time
           // egoCar.update_save_states(trajectory.targetState_s,
           // trajectory.targetState_d);

           // convert this trajectory in the s-d frame to to discrete XY points
           // the simulator can understand
           TrajectoryJMT traj = behavior.getPlanningResult();
           TrajectoryXY traj_xy = traj.trajectory;
           prev_path_sd = traj.path_sd;

           if (traj_xy.pts.xs.size() > 0) {
             XYPoints NextXY_points;
             NextXY_points = traj_xy.pts;
             NextXY_points.n = traj_xy.pts.n;

             //for (int i = 0; i < NextXY_points.n; i++)
             //    cout << "x = " << NextXY_points.xs[i] << "\ty = " << NextXY_points.ys[i] << endl;
/* 
            log_.write("***** New path  *****");
            for (int i = 0; i < NextXY_points.xs.size(); i++) {
                log_.of_ << "\tx = " << NextXY_points.xs[i]  << "\ty = " << NextXY_points.ys[i] << endl;
            }
            log_.write("***** *************  *****");
 */
             prev_path_xy.pts.xs = NextXY_points.xs;
             prev_path_xy.pts.ys = NextXY_points.ys;
             prev_path_xy.pts.n = prev_path_xy.pts.xs.size();
          }
        

          //*********************************
          //* Send updated path plan to simulator
          //*********************************

          json msgJson;
          msgJson["next_x"] = prev_path_xy.pts.xs;
          msgJson["next_y"] = prev_path_xy.pts.ys;

          auto msg = "42[\"control\"," + msgJson.dump() + "]";

          // this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          clock_gettime(CLOCK_MONOTONIC, &tend);
          double t_diff = (((double)tend.tv_sec + 1.0e-9 * tend.tv_nsec) -
                           ((double)tstart.tv_sec + 1.0e-9 * tstart.tv_nsec)) *
                          1000;

          // cout << "Process message duration: " << t_diff << "ms" << endl;
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}
