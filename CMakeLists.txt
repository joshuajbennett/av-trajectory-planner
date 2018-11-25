project(av-trajectory-planner CXX)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories("${PROJECT_SOURCE_DIR}/include")

find_package(PkgConfig REQUIRED)

pkg_check_modules(JSONCPP REQUIRED jsoncpp)

add_library(AvTrajectoryPlanner SHARED src/lib/AvTrajectoryPlanner.cpp)

target_include_directories(AvTrajectoryPlanner PRIVATE ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(AvTrajectoryPlanner ${JSONCPP_LIBRARIES})

add_executable(tests src/test/main.cpp)

target_include_directories(tests PRIVATE ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(tests AvTrajectoryPlanner ${JSONCPP_LIBRARIES})

