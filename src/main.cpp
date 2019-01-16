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


using namespace std;

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

  cfg.Load("config.json");
  cout << cfg.numLanes() << endl;
  cout << "Speed Limit: " << cfg.speedLimit() << endl;
  cout << "Track length: " << cfg.trackLength() << endl;
  cout << "Loading map...";
  map.LoadData(map_file_);
  cout << "Map loaded" << endl;

  // map.TestClosestWaypoint();

  // map.plot();

  h.onMessage([&cfg, &map, &just_starting](uWS::WebSocket<uWS::SERVER> ws,
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
    static TrajectorySD prev_path_sd;

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

          // Main car's localization Data
          CarLocalizationData vLocal(j[1]["x"], j[1]["y"], j[1]["s"], j[1]["d"],
                                     j[1]["yaw"], j[1]["speed"]);

          // Ergo car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];

          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Our default is to just give back our previous plan
          int n = previous_path_x.size();
          // XYPoints XY_points = {previous_path_x, previous_path_y, n};
          TrajectoryXY prev_path_xy(previous_path_x, previous_path_y);

          /////////////////////////////////////////
          // Sensor Fusion
          /////////////////////////////////////////

          // Sensor Fusion Data, a list of all other cars on the same side of
          // the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          /*
           if (cfg.dbgMain() == 1) {
            


             std::cout << "t: " << t_msg << ", dt: " << dt << "ms, x: " <<
           vLocal.x
                       << ", y: " << vLocal.y << ", s: " << vLocal.s << ", v:"
           << vLocal.speed << std::endl;
            


           } */
          t_oldmsg = t_msg;

          Vehicle egoCar(100, cfg);
          egoCar.update_position(car_s, car_d);
          egoCar.updatePositionXY(car_x, car_y);
          egoCar.update_speed(car_speed);
          egoCar.specify_adjacent_lanes();
          egoCar.front_gap = 10000; // TODO: REMOVE !!!!!
          // if (call_cnt > 3)
          // exit(-1);
          /////////////////////////////////////////
          // Localization
          /////////////////////////////////////////

          /////////////////////////////////////////
          // Trajectory
          /////////////////////////////////////////

          /////////////////////////////////////////
          // Prediction
          /////////////////////////////////////////

          /////////////////////////////////////////
          // Behavior Planning
          /////////////////////////////////////////

          if (just_starting) {
            // Our car hasn't moved yet. Let's move it!
            cout << "Starting engine..." << endl;
            prev_path_xy = egoCar.startEngine(map);
            just_starting = false;
            cout << "Engine started..." << endl;

            Trajectory trajectory(cfg, map);
            TrajectoryJMT prev_traj =
                trajectory.generateColdStartPrevPath(egoCar.s, egoCar.d);
            prev_path_sd = prev_traj.path_sd;

          } else if (n < cfg.pathSizeCutOff()) {
            // Our previous plan is about to run out, so append to it
            // Make a list of all relevant information about other cars
            vector<Vehicle> otherCars;

            for (int i = 0; i < sensor_fusion.size(); i++) {
              int id = sensor_fusion[i][0];
              double s = sensor_fusion[i][5];
              double d = sensor_fusion[i][6];
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];

              Vehicle car(id, cfg);
              car.update_position(s, d);
              car.update_speed(sqrt(vx * vx + vy * vy));
              otherCars.emplace_back(car);
            }

            // Print for debugging
            cout << "---------------------------------" << endl;
            cout << "STATE: s, d --- x, y --- v:" << endl;
            cout << car_s << " , " << car_d << " --- " << car_x << " , "
                 << car_y << " --- " << car_speed << ":" << endl;

            cout << "---------------------------------" << endl;
            cout << "our left:  our lane:   our right:" << endl;
            // print_lane(myCar.lane_at_left);
            // print_lane(myCar.lane);
            // print_lane(myCar.lane_at_right);
            cout << endl;

            cout << "---------------------------------" << endl;

            Trajectory trajectory(egoCar, BehaviorType::KEEPLANE, map, cfg);

            // Update saved state of our car (THIS IS IMPORTANT) with the latest
            // generated target states, this is to be used as the starting state
            // when generating a trajectory next time
            // egoCar.update_save_states(trajectory.targetState_s,
            // trajectory.targetState_d);

            // convert this trajectory in the s-d frame to to discrete XY points
            // the simulator can understand
            // int num_points = cfg.traverseTime()/cfg.timeIncrement();
            // XYPoints NextXY_points = map.makePath(
            //  trajectory.get_jmt_s(), trajectory.get_jmt_d(),
            //  cfg.timeIncrement(), num_points);

            // TrajectoryXY traj_xy =
            // trajectory.generateSplineBasedTrajectory(egoCar, XY_points);
            int num_points = cfg.traverseTime() /
                             cfg.timeIncrement(); // TODO: decide if we use this
                                                  // or the number of points
                                                  // defined in the cfg file
            TrajectoryXY traj_xy = trajectory.generateTrajectory(
                egoCar, BehaviorType::KEEPLANE, prev_path_xy, num_points);
            XYPoints NextXY_points;
            NextXY_points = traj_xy.pts;
            NextXY_points.n = traj_xy.pts.n;

            // Append these generated points to the old points
            prev_path_xy.pts.xs.insert(prev_path_xy.pts.xs.end(),
                                       NextXY_points.xs.begin(),
                                       NextXY_points.xs.end());

            prev_path_xy.pts.ys.insert(prev_path_xy.pts.ys.end(),
                                       NextXY_points.ys.begin(),
                                       NextXY_points.ys.end());

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
