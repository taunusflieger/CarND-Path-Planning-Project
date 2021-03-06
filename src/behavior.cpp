// =================================================================
//  Based on the current situation (egocar, othercars) within a
//  given planning horizon creates options for actions. The options
//  are weighted using a cost function. The bahavior system evaluates
//  these options and chooses the least cost option to define a
//  new target for the egocar
// =================================================================
#include "behavior.h"
#include "cost.h"
#include "log.h"
#include "trajectory.h"

double deg2rad(double x) { return x * M_PI / 180; }

extern Logger log_;

#define COST_THRESHHOLD 1000

const char *Behavior::StateNames[] = {"Ready", "LaneKeep", "LaneChangeLeft",
                                      "LaneChangeRight", "PrepareLaneChange"};

static FSM::Fsm<Behavior::States, Behavior::States::Ready, Behavior::Triggers>
    fsm;

Behavior::Triggers Behavior::currentTrigger_ = Behavior::Triggers::Initialize;
LaneType Behavior::target_lane_ = LaneType::UNSPECIFIED;

// Check if this approach works: in case we need to abort a lane change we reuse the previously generated trajectory
TrajectoryJMT Behavior::resulting_trajectory_;

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
                        { States::PrepareLaneChange  ,States::LaneKeep          ,Triggers::LaneChangeNotDesireable  ,[&]{return true;}  ,[&]{PerformLaneKeep(otherCars, egoCar, predictions);}},
                        { States::LaneChangeLeft     ,States::LaneChangeLeft    ,Triggers::NewPredictions  ,[&]{return true;}  ,[&]{PerformLaneChangeLeft(otherCars, egoCar, predictions);}},
                        { States::LaneChangeLeft     ,States::LaneKeep          ,Triggers::LaneChangeCompleted  ,[&]{return true;}  ,[&]{PerformLaneKeep(otherCars, egoCar, predictions);}},
                        { States::LaneChangeRight    ,States::LaneChangeRight   ,Triggers::NewPredictions  ,[&]{return true;}  ,[&]{PerformLaneChangeRight(otherCars, egoCar, predictions);}},
                        { States::LaneChangeRight    ,States::LaneKeep          ,Triggers::LaneChangeCompleted  ,[&]{return true;}  ,[&]{PerformLaneKeep(otherCars, egoCar, predictions);}}});
  // clang-format on

  log_.of_ << "==== Behavior::Behavior  path_sd.size() = " << prev_path.sd.path_s.size() << endl;
  // Execute state machine and process state resulting from trigger processing
  fsm.execute(currentTrigger_);
}

void Behavior::PerformPrepLaneChange(std::vector<Vehicle> &otherCars,
                                     Vehicle &egoCar,
                                     Prediction &predictions) {
  vector<TrajectoryCandidate> traj_candidates;
  log_.of_ << "==== Behavior::PerformPrepLaneChange ====" << endl;

  log_.of_ << "EgoCar is on lane " << static_cast<int>(egoCar.lane) << "speed = " << egoCar.v << endl;
  // identify lanes to which we can transition
  vector<LaneType> alternate_lanes;
  switch (egoCar.lane) {
    case LaneType::LEFT:
      alternate_lanes.push_back(LaneType::MID);
      break;
    case LaneType::RIGHT:
      alternate_lanes.push_back(LaneType::MID);
      break;
    case LaneType::MID:
      alternate_lanes.push_back(LaneType::RIGHT);
      alternate_lanes.push_back(LaneType::LEFT);
      break;
    default:
      assert(false);  // Egocar is not on a lane
      break;
  }

  vector<int> nearbycars = predictions.getNearbyCars(otherCars);
  for (auto const &target_lane : alternate_lanes) {
    // Create trajectories for target lane(s)
    TrajectoryCandidate tc;

    tc.t.lane = target_lane;
    tc.t.time = cfg_.traverseTime();

    // We don't accelerate during lane change
    // TODO: check if we can accelerate in JMT
    tc.t.velocity = egoCar.v;
    tc.t.acceleration = 1.0;

    Trajectory trajectory(cfg_, map_);
    tc.jmt_traj = trajectory.generate_trajectory_jmt(tc.t, map_, prev_path_);

    Cost cost(cfg_);
    tc.cost = cost.getCost(tc, predictions, otherCars, egoCar);

    log_.of_ << "Trajectory target lane = " << static_cast<int>(target_lane) << "\tcost = " << tc.cost << endl;
    traj_candidates.push_back(tc);
  }

  double min_cost = std::numeric_limits<double>::max();

  TrajectoryJMT jmt_traj;

  for (auto const &tc : traj_candidates) {
    if (tc.cost < min_cost) {
      min_cost = tc.cost;
      jmt_traj = tc.jmt_traj;
      target_lane_ = tc.t.lane;
    }
  }

  if (min_cost < COST_THRESHHOLD) {
    log_.of_ << "Found optimal trajectory lane = " << static_cast<int>(target_lane_) << "\tcost = " << min_cost << endl;

    resulting_trajectory_ = jmt_traj;
    log_.of_ << "path_sd.size() = " << resulting_trajectory_.path_sd.path_s.size() << endl;
    if (static_cast<int>(target_lane_) < static_cast<int>(egoCar.lane)) {
      currentTrigger_ = Triggers::ChangeToLeft;
      log_.of_ << "Triggers::ChangeToLeft" << endl;
    } else {
      currentTrigger_ = Triggers::ChangeToRight;
      log_.of_ << "Triggers::ChangeToRight" << endl;
    }
  } else {
    // no suitable lane found. Stay on current lane
    currentTrigger_ = Triggers::LaneChangeNotDesireable;
    log_.of_ << "Triggers::LaneChangeNotDesireable" << endl;
  }
  log_.write("***** ======== *****");
}

void Behavior::PerformReady(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                            Prediction &predictions){};

void Behavior::PerformLaneKeep(std::vector<Vehicle> &otherCars, Vehicle &egoCar,
                               Prediction &predictions) {
  double acceleration = cfg_.acceleration();
  bool prepLaneChange = false;
  ref_speed_ = cfg_.targetSpeed();

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
      //
      // TODO: set acceleration as a function of distance (progressive)
      //
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
  if (prepLaneChange)
    currentTrigger_ = Triggers::SlowCarAhead;
  else
    currentTrigger_ = Triggers::NewPredictions;
}

void Behavior::PerformLaneChangeLeft(std::vector<Vehicle> &otherCars,
                                     Vehicle &egoCar,
                                     Prediction &predictions) {
  log_.of_ << "==== Behavior::PerformLaneChangeLeft prev_path_.num_xy_reused = " << prev_path_.num_xy_reused << endl;
  // At this point it is mainly a placeholder ensuring
  // that we finish the lane change prior to calculating a new trajectory
  // We define the period of a lane change as a black out period. This
  // is clearly not save, but a simplification

  double target_d = egoCar.convert_lane_to_d(target_lane_);
  if (egoCar.d > (target_d - 0.2) && egoCar.d < (target_d + 0.2))
    currentTrigger_ = Triggers::LaneChangeCompleted;
  else
    currentTrigger_ = Triggers::NewPredictions;

  Trajectory trajectory(cfg_, map_);

  Target t;
  t.lane = target_lane_;
  t.time = cfg_.traverseTime();
  t.velocity = egoCar.v;
  t.acceleration = 1.0;
  resulting_trajectory_ = trajectory.generate_trajectory_jmt(t, map_, prev_path_);

  log_.write("***** ======== *****");
}

void Behavior::PerformLaneChangeRight(std::vector<Vehicle> &otherCars,
                                      Vehicle &egoCar,
                                      Prediction &predictions) {
  log_.of_ << "==== Behavior::PerformLaneChangeRight prev_path_.num_xy_reused = " << prev_path_.num_xy_reused << endl;
  // At this point it is mainly a placeholder ensuring
  // that we finish the lane change prior to calculating a new trajectory
  // We define the period of a lane change as a black out period. This
  // is clearly not save, but a simplification

  double target_d = egoCar.convert_lane_to_d(target_lane_);
  if (egoCar.d > (target_d - 0.2) && egoCar.d < (target_d + 0.2))
    currentTrigger_ = Triggers::LaneChangeCompleted;
  else
    currentTrigger_ = Triggers::NewPredictions;

  Trajectory trajectory(cfg_, map_);

  Target t;
  t.lane = target_lane_;
  t.time = cfg_.traverseTime();
  t.velocity = egoCar.v;
  t.acceleration = 1.0;
  resulting_trajectory_ = trajectory.generate_trajectory_jmt(t, map_, prev_path_);

  log_.write("***** ======== *****");
}

