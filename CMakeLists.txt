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

add_subdirectory(pybind11)

find_package(PythonInterp 2.7)
message(${PYTHON_EXECUTABLE})

# Need to search for python executable again to pick up an activated
# virtualenv python, if any.
unset(PYTHON_EXECUTABLE CACHE)
find_program(PYTHON_EXECUTABLE python
      PATHS ENV PATH         # look in the PATH environment variable
      NO_DEFAULT_PATH        # do not look anywhere else...
      )
# Make FindNumPy available
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")
find_package(NumPy REQUIRED)

pybind11_add_module(AvTrajectoryPlannerPython src/lib_python/AvTrajectoryPlannerPython.cpp)
target_include_directories(AvTrajectoryPlannerPython PRIVATE ${NUMPY_INCLUDES})
target_link_libraries(AvTrajectoryPlannerPython PRIVATE AvTrajectoryPlanner)
set_target_properties(AvTrajectoryPlannerPython
  PROPERTIES
  OUTPUT_NAME MatterSim)

