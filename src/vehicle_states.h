#include <vector>
using std::vector;


/**
 * Possible states of the vehicle in the high way traffic mode
 */
enum class VehicleState {
  LCL,  // lane change left
  PLCL, // prepare lane change left
  KL,   // keep lane
  PLCR, // prepare lane change right
  LCR  // lane change right
};


int label_vehicle_state(VehicleState state) {
  switch(state) {
    case VehicleState::KL: return 0;
    case VehicleState::LCL: return -2;
    case VehicleState::LCR: return 2;
    case VehicleState::PLCL: return -1;
    case VehicleState::PLCR: return 1;
  }
}

int get_final_lane(int lane, VehicleState state) {
  if (state == VehicleState::KL ) {
    return lane;
  }
  
  if (state == VehicleState::LCL || state == VehicleState::PLCL) {
    if (lane == 0) {
      return lane;
    } else {
      return lane - 1;
    }
  }
 
  if (state == VehicleState::LCR || state == VehicleState::PLCR) {
    if (lane == 2) {
      return lane;
    } else {
      return lane + 1;
    }
  }
}


/**
 * Possible states that vehicle can transit to from the current state
 */
std::vector<VehicleState> possible_next_states(VehicleState state) {
  
  std::vector<VehicleState> states;
  states.push_back(VehicleState::KL);
  if (state == VehicleState::KL) {
    states.push_back(VehicleState::PLCL);
    states.push_back(VehicleState::PLCR);
  } 

  if (state == VehicleState::PLCL) {
    states.push_back(VehicleState::PLCL);
    states.push_back(VehicleState::LCL);
  }
  
  if (state == VehicleState::PLCR) {
    states.push_back(VehicleState::PLCR);
    states.push_back(VehicleState::LCR);
  }
  
  return states;
}