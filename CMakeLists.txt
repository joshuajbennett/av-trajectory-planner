project(av-trajectory-planner CXX)
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories("${PROJECT_SOURCE_DIR}/include")

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

find_package(xtensor REQUIRED)

add_library(IterativeLQR SHARED src/lib_cpp/IterativeLQR.cpp)

target_include_directories(IterativeLQR PRIVATE ${xtensor_INCLUDE_DIRS})
target_link_libraries(IterativeLQR xtensor)

add_library(AvTrajectoryPlanner SHARED src/lib_cpp/AvTrajectoryPlanner.cpp)

target_include_directories(AvTrajectoryPlanner PRIVATE ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(AvTrajectoryPlanner ${JSONCPP_LIBRARIES} IterativeLQR)

add_executable(tests src/test/main.cpp)

target_include_directories(tests PRIVATE ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(tests AvTrajectoryPlanner ${JSONCPP_LIBRARIES})

add_subdirectory(pybind11)

# Make FindNumPy available
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
find_package(NumPy REQUIRED)

pybind11_add_module(AvTrajectoryPlannerPython src/lib_python/AvTrajectoryPlannerPython.cpp)
target_include_directories(AvTrajectoryPlannerPython PRIVATE ${NUMPY_INCLUDES})
target_link_libraries(AvTrajectoryPlannerPython PRIVATE AvTrajectoryPlanner)
set_target_properties(AvTrajectoryPlannerPython
  PROPERTIES
  OUTPUT_NAME AvTrajectoryPlanner)

