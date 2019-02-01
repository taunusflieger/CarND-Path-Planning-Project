// =================================================================
//  Based on the current situation (egocar, othercars) within a
//  given planning horizon creates options for actions. The options
//  are weighted using a cost function. The bahavior system evaluates
//  these options and chooses the least cost option to define a
//  new target for the egocar
// =================================================================
#include "log.h"
#include "trajectory.h"
#include <behavior.h>

extern Logger log_;

const char *Behavior::StateNames[] = {"Ready", "LaneKeep", "LaneChangeLeft",
                                      "LaneChangeRight", "PrepareLaneChange"};

static FSM::Fsm<Behavior::States, Behavior::States::Ready, Behavior::Triggers>
    fsm;

Behavior::Triggers Behavior::currentTrigger_ = Behavior::Triggers::Initialize;

Behavior::Behavior(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                   Prediction &predictions, PreviousPath &prev_path, Map &map,
                   Config &cfg)
    : egoCar_(egoCar), cfg_(cfg), map_(map), prev_path_(prev_path) {
  // clang-format off
  fsm.add_transitions({
                        //  from state ,to state  ,triggers        ,guard                    ,action
                        { States::Ready              ,States::LaneKeep          ,Triggers::Initialize       ,[&]{return true;}  ,[&]{PerformLaneKeep(otherCars, egoCar, predictions);}},
                        { States::LaneKeep           ,States::LaneKeep          ,Triggers::NewPredictions   ,[&]{return true;}  ,[&]{PerformLaneKeep(otherCars, egoCar, predictions);}},
                        { States::LaneKeep           ,States::PrepareLaneChange ,Triggers::SlowCarAhead  ,[&]{return true;}  ,[&]{PerformPrepLaneChange(otherCars, egoCar, predictions);}},
                        { States::PrepareLaneChange  ,States::PrepareLaneChange ,Triggers::NewPredictions  ,[&]{return true;}  ,[&]{PerformPrepLaneChange(otherCars, egoCar, predictions);}},
                        { States::PrepareLaneChange  ,States::LaneChangeLeft    ,Triggers::ChangeToLeft  ,[&]{return true;}  ,[&]{PerformLaneChangeLeft(otherCars, egoCar, predictions);}},
                        { States::PrepareLaneChange  ,States::LaneChangeRight   ,Triggers::ChangeToRight  ,[&]{return true;}  ,[&]{PerformLaneChangeRight(otherCars, egoCar, predictions);}},
                        { States::LaneChangeLeft     ,States::LaneChangeLeft    ,Triggers::NewPredictions  ,[&]{return true;}  ,[&]{PerformLaneChangeLeft(otherCars, egoCar, predictions);}},
                        { States::LaneChangeLeft     ,States::LaneKeep          ,Triggers::LaneChangeCompleted  ,[&]{return true;}  ,[&]{PerformLaneKeep(otherCars, egoCar, predictions);}},
                        { States::LaneChangeRight    ,States::LaneChangeRight   ,Triggers::NewPredictions  ,[&]{return true;}  ,[&]{PerformLaneChangeRight(otherCars, egoCar, predictions);}},
                        { States::LaneChangeRight    ,States::LaneKeep          ,Triggers::LaneChangeCompleted  ,[&]{return true;}  ,[&]{PerformLaneKeep(otherCars, egoCar, predictions);}}});
  // clang-format on

  // Execute state machine and process state resulting from trigger processing
  fsm.execute(currentTrigger_);
}

void Behavior::PerformPrepLaneChange(std::vector<Vehicle> &otherCars,
                                     Vehicle &egoCar,
                                     Prediction &predictions){};

void Behavior::PerformReady(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                            Prediction &predictions){};

void Behavior::PerformLaneKeep(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                               Prediction &predictions) {
  double acceleration = cfg_.acceleration();
  bool prepLaneChange = false;
  ref_speed_ = cfg_.targetSpeed();
  currentTrigger_ = Triggers::NewPredictions;
  Target t;

  int idx_carAhead =
      predictions.getNearbyCars(otherCars)[static_cast<int>(egoCar.lane) * 2];

  if (idx_carAhead >= 0) {
    log_.of_ << "==== Behavior::PerformLaneKeep  car ahead ====" << endl;
    // We have a car in our lane in front of us
    front_gap_ = otherCars[idx_carAhead].front_gap;
    if (front_gap_ <= cfg_.collisionBuffer()) {
      ref_speed_ = otherCars[idx_carAhead].v >= cfg_.targetSpeed()
                       ? cfg_.targetSpeed()
                       : otherCars[idx_carAhead].v - 2.0;
    }

    if (ref_speed_ <= egoCar.v) {
      prepLaneChange = true;
      if (front_gap_ <= cfg_.collisionBuffer() / 2) {
        // harder deceleration
        acceleration = -cfg_.acceleration() / 2;
      } else if (front_gap_ <= cfg_.collisionBuffer() / 3) {
        // emergency break
        acceleration = -cfg_.acceleration();
      } else
        acceleration = -cfg_.acceleration() / 4;
      log_.of_ << "----- speed acceleration = " << acceleration << endl;
    } else {
      acceleration = cfg_.acceleration() / 4;
      log_.of_ << "+++++ speed acceleration = " << acceleration << endl;
    }

    log_.of_ << "Front car speed: " << ref_speed_ << "\tdist = " << front_gap_ << "\ttarget Speed = " << cfg_.targetSpeed() 
             << endl;
  }

  t.velocity = ref_speed_;
  t.time = cfg_.traverseTime();
  t.lane = egoCar_.lane;
  t.acceleration = acceleration;

  // Calculate trajectory
  TrajectoryJMT jmtTj;
  Trajectory trajectory(cfg_, map_);
  resulting_trajectory_ =
      trajectory.generateSDTrajectory(egoCar, prev_path_, t);

  // Check if we need to prepare a lane change
  if (prepLaneChange) {
    // Find lanes which are free

    // generate trajectories

    // 

  }
};

void Behavior::PerformLaneChangeLeft(std::vector<Vehicle> &otherCars,
                                     Vehicle &egoCar,
                                     Prediction &predictions){};

void Behavior::PerformLaneChangeRight(std::vector<Vehicle> &otherCars,
                                      Vehicle &egoCar,
                                      Prediction &predictions){};

void Behavior::dbg_fsm(States from_state, States to_state, Triggers trigger) {
  if (from_state != to_state) {
    cout << "State Changed To: " << StateNames[to_state] << endl;
  }
}
