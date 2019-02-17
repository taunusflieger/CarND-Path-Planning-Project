#pragma once

#include "fsm.h"
#include "prediction.h"
#include "map.h"
#include "types.h"


class Behavior {
public:
  enum class States : int {
    Ready = 0,
    LaneKeep = 1,
    LaneChangeLeft = 2,
    LaneChangeRight = 3,
    PrepareLaneChange = 4

  };
  enum class Triggers {
    None,
    Initialize,
    SlowCarAhead,
    NewPredictions,
    ChangeToRight,
    ChangeToLeft,
    LaneChangeCompleted,
    LaneChangeNotDesireable
  };

  Behavior(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
           Prediction &predictions, PreviousPath &prev_path, Map& map, Config &cfg);

  TrajectoryJMT getPlanningResult(void) { return resulting_trajectory_; };

  void dbg_fsm(States from_state, States to_state, Triggers trigger);

private:
  // This contains the trajectory which represents the result of the planning
  // process
  TrajectoryJMT resulting_trajectory_;
  PreviousPath &prev_path_;
  Map &map_;
  static const char *StateNames[];
  static Triggers currentTrigger_;
  static LaneType target_lane_;
  double ref_speed_;
  double ref_accel_;
  double front_gap_;

  void PerformPrepLaneChange(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                             Prediction &predictions);
  void PerformReady(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                    Prediction &predictions);
  void PerformLaneKeep(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                       Prediction &predictions);
  void PerformLaneChangeLeft(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                             Prediction &predictions);
  void PerformLaneChangeRight(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                              Prediction &predictions);

  Config &cfg_;
  Vehicle &egoCar_;
};