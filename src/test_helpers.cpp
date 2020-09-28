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
