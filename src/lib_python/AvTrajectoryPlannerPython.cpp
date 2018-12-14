#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include "AvStructs.hpp"
#include "AvTrajectoryPlanner.hpp"
#include <iostream>
#include <numpy/ndarrayobject.h>
#include <numpy/npy_math.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using namespace av_trajectory_planner;
using namespace av_structs;

PYBIND11_MODULE(AvTrajectoryPlanner, m)
{
	py::class_<SolverParams>(m, "SolverParams")
		.def(py::init<>())
		.def(py::init<double, double, double, unsigned long, bool, bool>())
		.def_readwrite("max_time", &SolverParams::max_time)
		.def_readwrite("solver_dt", &SolverParams::solver_dt)
		.def_readwrite("max_iterations", &SolverParams::max_iterations)
		.def_readwrite("constrain_velocity", &SolverParams::constrain_velocity)
		.def_readwrite("constrain_steering_angle", &SolverParams::constrain_steering_angle);
	py::class_<AvState>(m, "AvState")
		.def(py::init<>())
		.def(py::init<double, double, double, double, double>())
		.def_readwrite("x", &AvState::x)
		.def_readwrite("y", &AvState::y)
		.def_readwrite("psi", &AvState::psi)
		.def_readwrite("delta_f", &AvState::delta_f)
		.def_readwrite("vel_f", &AvState::vel_f);
	py::class_<AvAction>(m, "AvAction")
		.def(py::init<>())
		.def(py::init<double, double>())
		.def_readwrite("turn_rate", &AvAction::turn_rate)
		.def_readwrite("accel_f", &AvAction::accel_f);
	py::class_<AvParams>(m, "AvParams")
		.def(py::init<>())
		.def(py::init<double, double, double, double, double>())
		.def_readwrite("l_r", &AvParams::l_r)
		.def_readwrite("l_f", &AvParams::l_r)
		.def_readwrite("max_delta_f", &AvParams::max_delta_f)
		.def_readwrite("max_accel_f", &AvParams::max_accel_f)
		.def_readwrite("max_vel_f", &AvParams::max_vel_f);
	py::class_<Point>(m, "Point")
		.def(py::init<>())
		.def(py::init<double, double>())
		.def_readwrite("x", &Point::x)
		.def_readwrite("y", &Point::y);
	py::class_<Boundary>(m, "Boundary")
		.def(py::init<>())
		.def(py::init<std::vector<Point>>())
		.def_readwrite("vertices", &Boundary::vertices);
	py::class_<Pose>(m, "Pose")
		.def(py::init<>())
		.def(py::init<double, double, double>())
		.def_readwrite("x", &Pose::x)
		.def_readwrite("y", &Pose::y)
		.def_readwrite("psi", &Pose::psi);
	py::class_<ObstacleTrajectory>(m, "ObstacleTrajectory")
		.def(py::init<>())
		.def(py::init<Boundary, std::vector<Pose>, double>())
		.def_readwrite("outline", &ObstacleTrajectory::outline)
		.def_readwrite("table", &ObstacleTrajectory::table)
		.def_readwrite("dt", &ObstacleTrajectory::dt)
		.def("interpolate", &ObstacleTrajectory::interpolate);
	py::class_<ObstacleStatic>(m, "ObstacleStatic")
		.def(py::init<>())
		.def(py::init<Boundary, Pose>())
		.def_readwrite("outline", &ObstacleStatic::outline)
		.def_readwrite("obs_pose", &ObstacleStatic::obs_pose);
	py::class_<AvTrajectory>(m, "AvTrajectory")
		.def(py::init<>())
		.def(py::init<Boundary, std::vector<AvState>, double, AvParams>())
		.def_readwrite("outline", &AvTrajectory::outline)
		.def_readwrite("table", &AvTrajectory::table)
		.def_readwrite("dt", &AvTrajectory::dt)
		.def_readwrite("av_parameters", &AvTrajectory::av_parameters)
		.def("interpolate", &AvTrajectory::interpolate);
	py::class_<Planner>(m, "Planner")
		.def(py::init<>())
		.def(py::init<AvState, AvState, AvParams, Boundary, SolverParams>())
		.def("setGoal", &Planner::setGoal)
		.def("getGoal", &Planner::getGoal)
		.def("setInitialState", &Planner::setInitialState)
		.def("getInitialState", &Planner::getInitialState)
		.def("clearObstacles", &Planner::clearObstacles)
		.def("appendObstacleTrajectory", &Planner::appendObstacleTrajectory)
		.def("appendObstacleStatic", &Planner::appendObstacleStatic)
		.def("getObstacleTrajectories",
			 &Planner::getObstacleTrajectories,
			 py::return_value_policy::take_ownership)
		.def("setSolverMaxTime", &Planner::setSolverMaxTime)
		.def("setSolverTimeStep", &Planner::setSolverTimeStep)
		.def("setSolverEpsilon", &Planner::setSolverEpsilon)
		.def("setSolverMaxIterations", &Planner::setSolverMaxIterations)
		.def("enableVelocityConstraint()", &Planner::enableVelocityConstraint)
		.def("disableVelocityConstraint()", &Planner::disableVelocityConstraint)
		.def("enableSteeringConstraint()", &Planner::enableSteeringConstraint)
		.def("disableSteeringConstraint()", &Planner::disableSteeringConstraint)
		.def("solveTrajectory", &Planner::solveTrajectory, py::return_value_policy::take_ownership)
		.def("saveToJson", &Planner::saveToJson, py::return_value_policy::take_ownership)
		.def("loadFromJson", &Planner::loadFromJson);
}
