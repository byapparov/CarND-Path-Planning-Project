#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "lanes_state_machine.h"


TEST_CASE( "Can calculate lane number correctly", "[frenet_to_lane_number]") {
  REQUIRE( frenet_to_lane_number(1.0) == 0 );
  REQUIRE( frenet_to_lane_number(2.0) == 0 );
  REQUIRE( frenet_to_lane_number(3.0) == 0 );
  
  REQUIRE( frenet_to_lane_number(6.0) == 1 );
  REQUIRE( frenet_to_lane_number(8.0) == 2 );
  
  // position is outside of the road
  REQUIRE(frenet_to_lane_number(-1) == -1 );
  REQUIRE(frenet_to_lane_number(13) == -1 );
}

TEST_CASE( "Can calculate Fernet d coordinate from lane nubmer", "[frenet_to_lane_number]") {
  REQUIRE ( lane_number_to_frenet(0) == 2);
  REQUIRE ( lane_number_to_frenet(1) == 6);
  
}


TEST_CASE( "Slow speed of the car is penalised", "[car_speed_cost]") {
  REQUIRE( car_speed_cost(100, 80) == 0.2);
  REQUIRE( car_speed_cost(80, 100) == 0);
}

TEST_CASE( "Cost of the target position determined for a given sensor fustion", "[target_lane_speed_cost]") {
  
  double s = 100;
  double d = 6;
  double v = 47.5; // velocity target
  double delta_t = 0.5;
  
  vector<vector <double>> sensor_fusion;
  vector <double> car_1 = {
    1,   // id
    543, // x
    532, // y
    30, // xv
    0.1, // xy
    120,
    6.2 // lane 2
  };
  
  sensor_fusion.push_back(car_1);
    
  
  double res = target_lane_speed_cost(s, d, v, delta_t, sensor_fusion);
  double exp = 30.0 / 47.5;
  
  REQUIRE ( res == Approx(res));
  
  
  vector <double> car_2 = {
    2,   // id
    543, // x
    532, // y
    50, // xv
    0.1, // xy
    120,
    9.8 // land 3
  };
  
  sensor_fusion.push_back(car_2);

  // Car 2 is going in the same lane but at good spped
  res = target_lane_speed_cost(s, 9, v, delta_t, sensor_fusion);
  exp = 0;
  REQUIRE ( res == Approx(res));
  
  // There are no cars in lane 1
  res = target_lane_speed_cost(s, 2, v, delta_t, sensor_fusion);
  exp = 0;
  REQUIRE ( res == Approx(res));
  
  
  vector <double> car_3 = {
    1,   // id
    543, // x
    532, // y
    60, // xv
    0.1, // xy
    110, // s
    5.7 // lane 2
  };
  sensor_fusion.push_back(car_3);
  // Car 3 is moving at good speed before car 1. 
  // This should override the cost and set it to zero.
  res = target_lane_speed_cost(s, d, v, delta_t, sensor_fusion);
  exp = 0;
  REQUIRE ( res == Approx(res));
}

TEST_CASE( "state label to integer works", "[get_final_lane]") {
  REQUIRE (label_vehicle_state(VehicleState::KL) == 0);
  REQUIRE (label_vehicle_state(VehicleState::LCR) == 2);
}

TEST_CASE( "Final target lane is calculated successfully based on the current lane and state", "[get_final_lane]") {

  // Keep lane returns the same lane  
  REQUIRE ( get_final_lane(1, VehicleState::KL) == 1);
  REQUIRE ( get_final_lane(2, VehicleState::KL) == 2);
  
  // Lane change left reduced lane number
  REQUIRE ( get_final_lane(1, VehicleState::LCL) == 0);
  REQUIRE ( get_final_lane(0, VehicleState::LCL) == 0);
  
  // Lane change right increments lane number by one unless 
  // in the far right lane
  REQUIRE ( get_final_lane(0, VehicleState::LCR) == 1);
  REQUIRE ( get_final_lane(2, VehicleState::LCR) == 2);
}

TEST_CASE( "Next state of the vehicle is assesed correctly", "[get_final_lane]") {
  
  vector<VehicleState> states = possible_next_states(VehicleState::KL);
  REQUIRE (states.size() == 3); // Can KL / PLCL / PLCR
  
  states = possible_next_states(VehicleState::LCR);
  REQUIRE (states.size() == 1); 
  REQUIRE (states[0] == VehicleState::KL);
}