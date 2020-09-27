#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "helpers.h"


TEST_CASE( "My first test", "[hasData]") {
  REQUIRE( 1 == 1 );
}
