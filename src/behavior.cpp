// =================================================================
//  Based on the current situation (egocar, othercars) within a 
//  given planning horizon creates options for actions. The options
//  are weighted using a cost function. The bahavior system evaluates
//  these options and chooses the least cost option to define a
//  new target for the egocar
// =================================================================
#include "behavior.h"

Behavior::Behavior(std::vector<Vehicle>&  otherCars, Vehicle& egoCar,
             Prediction& predictions, Config& cfg) : egoCar_(egoCar), cfg_(cfg) {

}