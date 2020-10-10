#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "vehicle.h"


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

TEST_CASE( "Cost of the target position determined for a given sensor fustion", "[target_lane_safe_speed]") {
  
  double s = 100;
  double d = 6;
  double speed_limit = mph_to_ms(47.5); // ms
  double v = 47.5; // car velocity
  double delta_t = 0.5;
  
  vector<vector <double>> sensor_fusion;
  vector <double> car_1 = {
    1,   // id
    543, // x
    532, // y
    10, // xv
    0.1, // xy
    120, // s
    6.2 // lane 2
  };
  
  sensor_fusion.push_back(car_1);
    
  
  double res = target_lane_safe_speed(s, d, v, speed_limit, delta_t, sensor_fusion, 30);
  double exp = 10.0 - 0.25;
  
  REQUIRE ( abs(res - exp) < 0.2  );
  
  
  vector <double> car_2 = {
    2,   // id
    543, // x
    532, // y
    mph_to_ms(50), // xv
    0.1, // xy
    120,
    9.8 // land 3
  };
  
  sensor_fusion.push_back(car_2);

  // Car 2 is going in the same lane but at good spped
  exp = speed_limit;
  res = target_lane_safe_speed(s, 9, v, speed_limit, delta_t, sensor_fusion, 50);
  REQUIRE ( res == Approx(exp));
  
  // There are no cars in lane 1
  res = target_lane_safe_speed(s, 2, v, speed_limit, delta_t, sensor_fusion, 50);
  exp = speed_limit;
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
  res = target_lane_safe_speed(s, d, v, speed_limit, delta_t, sensor_fusion, 50);
  exp = speed_limit;
  REQUIRE ( res == Approx(exp));
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


TEST_CASE( "Lane switch costs 1", "[target_lane_switch_cost]") {
  REQUIRE (target_lane_switch_cost(6, 10) == 1);
  REQUIRE (target_lane_switch_cost(2, 10) == 1);
  
  // half lane switch half cost
  REQUIRE (target_lane_switch_cost(2, 4) == .5);
  
  // no switch no cost
  REQUIRE (target_lane_switch_cost(2, 2) == 0);
}

TEST_CASE( "Cost of colision is estimated", "[colision_cost]") {
  
  vector<vector <double> > predictions;
  
  // In this scenario the car is far away in the second lane
  vector<double> car_1 = { // car is in the right lane
    1, 
    100,
    5.5
  };
  predictions.push_back(car_1);
  
  vector<vector <double>> t;
  vector<double> ts, td;
  ts = {25, 40};
  td = {4, 5.5};
  t.push_back(ts);
  t.push_back(td);
  double res = colision_cost(t, 6, predictions);
  REQUIRE( res < 0.0001);
  t.clear();
  
  vector<double> car_2 = { // car is in the right lane
    2, 
    30,
    5.5
  };
  
  predictions.push_back(car_2);
  // Car is 10m ahead
  ts = {5, 21};
  td = {4, 6};
  t.push_back(ts);
  t.push_back(td);
  res = colision_cost(t, 6, predictions);
  t.clear();
  
    
  REQUIRE( res < .3 );
  REQUIRE( res > 0 );
  
  // Car 2 is inside the trajectory;
  ts = {20, 50};
  td = {4, 6};
  t.push_back(ts);
  t.push_back(td);
  res = colision_cost(t, 6, predictions);
  REQUIRE( res == 1 );
  t.clear();
  
  // Car 2 is with 10 meeters behind the trajectory;
  ts = {34, 50};
  td = {4, 6};
  t.push_back(ts);
  t.push_back(td);
  res = colision_cost(t, 6, predictions);
  REQUIRE( res == 1 );
  t.clear();
  
  // Adding car very close to another lane 
  // does not change cost
  vector<double> car_3 = { // car is in the left lane
    2, 
    15,
    2
  };
  predictions.push_back(car_3);
  ts = {0, 10};
  td = {4, 6};
  t.push_back(ts);
  t.push_back(td);
  res = colision_cost(t, 6, predictions);
  REQUIRE( res < 0.0001 );
  t.clear();
}


TEST_CASE ( "Prediction of based on sensor fusion is correct", "[predict_sensor_fusion]") {
  vector<vector <double>> sensor_fusion;
  vector <double> car_1 = {
    1,   // id
    543, // x
    532, // y
    10, // xv
    1, // xy
    120, // s
    6.2 // lane 2
  };
  
  sensor_fusion.push_back(car_1);
  
  vector<vector <double> > prediction;
  
  prediction = predict_sensor_fusion(sensor_fusion, 1);
  
  double v = sqrt(101);
  double s_predicted = 120 + v;
  
  REQUIRE ( prediction[0][1] == s_predicted);
  
 
  vector <double> car_2 = {
    1,   // id
    543, // x
    532, // y
    4, // xv
    3, // xy
    10, // s
    2 // left lane
  };
  sensor_fusion.push_back(car_2);
  
  prediction = predict_sensor_fusion(sensor_fusion, .1);
  REQUIRE (prediction.size() == 2);
  // speed is 5, so we expect .5 meters to be added in .1 seconds
  REQUIRE ( prediction[1][1] == 10.5);
  REQUIRE ( prediction[1][2] == 2); // we don't predict lanes
  
}

TEST_CASE ( "Map related transformations are correct", "[map_test]") {
  
  vector<double> res = LocalCoordinates(1970, 1190, atan2(1, 0), 1970, 1190);
  
  // Same point returns zero point
  REQUIRE ( res[0] == 0);
  REQUIRE ( res[1] == 0);
  
  res = LocalCoordinates(1970, 1190, atan2(0, 1), 1980, 1190);
  REQUIRE ( Approx(res[0]) == 10);
  REQUIRE ( Approx(res[1]) == 0);
  
  res = LocalCoordinates(1970, 1190, atan2(1, 0), 1980, 1190);
  REQUIRE ( res[1] == Approx(-10.0));
  
  
  res = LocalCoordinates(1970, 1190, atan2(1, 0), 1970, 1200);
  REQUIRE ( res[0] == Approx(10.0));
  
  res = GlobalCoordinates(1970, 1190, atan2(1, 0), 10, 0);
  REQUIRE ( res[0] == Approx(1970));
  REQUIRE ( res[1] == Approx(1200));
  
  
  res = GlobalCoordinates(1970, 1190, atan2(0, -1), 10, 0);
  REQUIRE ( res[0] == Approx(1960));
  REQUIRE ( res[1] == Approx(1190));
}