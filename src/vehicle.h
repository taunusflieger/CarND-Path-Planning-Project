#pragma once

#include "map.h"
#include "fsm.h"
#include "types.h"



class Vehicle
{
private:
    Map map_;
    vector<SensorFusionData> sensorFusionData_; 
    CarLocalizationData locData;
    
    enum States { Normal, Follow, ChangeLeft, ChangeRight };
    enum Triggers {  CarAhead, Clear };
    FSM::Fsm<States, States::Normal, Triggers> fsm_;
    static const std::string stateNames_[];
    

    void dbg_fsm(States from_state, States to_state, Triggers trigger);
    void printStatus(void);

public:
    Vehicle(const Map& map);
    ~Vehicle() {};

    void updateTrajectory(const CarLocalizationData& newloc, const std::vector<double>& previous_path_x, const std::vector<double>& previous_path_y);
    std::vector<SensorFusionData>& getSensorfusionData() { return sensorFusionData_; }
  
};

