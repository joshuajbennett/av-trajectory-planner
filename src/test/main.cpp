#define CATCH_CONFIG_MAIN // This tells Catch to provide a main() - only do this
                          // in one cpp file

#include <cmath>
#include <fstream>
#include <iostream>
#include <unordered_map>

#include <json/json.h>

#include "Catch.hpp"

#include "AvTrajectoryPlanner.hpp"

using namespace av_trajectory_planner;

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

double radians(double deg) { return deg * M_PI / 180.0; }

double degrees(double rad) { return (rad * 180.0) / M_PI; }

TEST_CASE("Test1", "[Actions]") {
  std::cout << "Running tests..." << std::endl;
}
