#include "vehicle.h"
#include <iostream>
#include <unistd.h>



const std::string  Vehicle::stateNames_[] =   {"Normal","Follow Vehicle In Front", "Change To Left Lane", "Change To Right Lane" };   

Vehicle::Vehicle(const Map& map, const Config& cfg) : map_(map), cfg_(cfg) {
    
  
}

void Vehicle::updateTrajectory(const CarLocalizationData& newloc,
                               const std::vector<double>& previous_path_x, 
                               const std::vector<double>& previous_path_y) {

    locData = newloc;
    

}

void Vehicle::dbg_fsm(States from_state, States to_state, Triggers trigger) {
    if (from_state != to_state) {
        std::cout << "State Changed To: " << stateNames_[to_state] << "\n";
    }
}

void Vehicle::printStatus(void)
{
    cout << "-----------------------" << endl;
    cout << "Speed: " << locData.speed << endl;
    cout << "x    : " << locData.x << endl;
    cout << "y    : " << locData.y << endl;
}

