// =================================================================
//  Based on the current situation (egocar, othercars) within a
//  given planning horizon creates options for actions. The options
//  are weighted using a cost function. The bahavior system evaluates
//  these options and chooses the least cost option to define a
//  new target for the egocar
// =================================================================
#include <behavior.h>
#include "log.h"
#include "trajectory.h"

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
  ref_speed_ = cfg_.targetSpeed();
  currentTrigger_ = Triggers::NewPredictions;
  Target t;

  int idx_carAhead =
      predictions.getNearbyCars(otherCars)[static_cast<int>(egoCar.lane) * 2];

  if (idx_carAhead >= 0) {
    // We have a car in our lane in front of us
    front_gap_ = otherCars[idx_carAhead].front_gap;
    if (front_gap_ <= cfg_.collisionBuffer()) {
      ref_speed_ = otherCars[idx_carAhead].v >= cfg_.targetSpeed() ? cfg_.targetSpeed() : otherCars[idx_carAhead].v - 2.0;
      
    }
    if (ref_speed_ <= egoCar.v)
        acceleration = -cfg_.acceleration() / 3;
      else
        acceleration = cfg_.acceleration() / 3;
    log_.of_ << "==== Behavior::PerformLaneKeep  car ahead ====" << endl;
    log_.of_ << "Front car speed: " << ref_speed_ << "\tdist = " << front_gap_
             << endl;
  }
  /*
  // If the car in front is going fast or we are very far from it anyway, go as
  // fast as we can Else let's go a notch slower than the car in front
  bool safe = (car.front_v > SPEED_LIMIT) || (car.front_gap > FRONT_BUFFER);
  double target_v = safe ? SPEED_LIMIT : (car.front_v - SPEED_BUFFER);

  // But if the car in front is too slow, let's go a little faster
  target_v = target_v > MIN_SPEED ? target_v : MIN_SPEED;
 */
  // Calculate target

  t.velocity = ref_speed_;
  t.time = cfg_.traverseTime();
  t.lane = egoCar_.lane;
  t.acceleration = acceleration;

  // Calculate trajectory
  TrajectoryJMT jmtTj;
  Trajectory trajectory(cfg_, map_);
  resulting_trajectory_ =
      trajectory.generateSDTrajectory(egoCar, prev_path_, t);

  /* resulting_trajectory_xy_ = trajectory.generateTrajectory(
      egoCar, BehaviorType::KEEPLANE, prev_path_xy_, t);
 */
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
