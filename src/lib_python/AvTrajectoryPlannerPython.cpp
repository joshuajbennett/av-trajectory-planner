#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include "AvTrajectoryPlanner.hpp"
#include <iostream>
#include <numpy/ndarrayobject.h>
#include <numpy/npy_math.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

using namespace av_trajectory_planner;

PYBIND11_MODULE(AvTrajectoryPlanner, m)
{
	py::class_<AvState>(m, "AvState")
		.def(py::init<>())
		.def(py::init<double, double, double, double, double>())
		.def_readwrite("x", &AvState::x)
		.def_readwrite("y", &AvState::y)
		.def_readwrite("psi", &AvState::psi)
		.def_readwrite("delta_f", &AvState::delta_f)
		.def_readwrite("vel_f", &AvState::vel_f);
	py::class_<AvParams>(m, "AvParams")
		.def(py::init<>())
		.def(py::init<double, double, double, double>())
		.def_readwrite("l_r", &AvParams::l_r)
		.def_readwrite("l_f", &AvParams::l_r)
		.def_readwrite("max_delta_f", &AvParams::max_delta_f)
		.def_readwrite("max_accel_f", &AvParams::max_accel_f);
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
		.def(py::init<AvState, AvState, AvParams, Boundary, double, double>())
		.def("setGoal", &Planner::setGoal)
		.def("setInitialState", &Planner::setInitialState)
		.def("clearObstacles", &Planner::clearObstacles)
		.def("appendObstacleTrajectory", &Planner::appendObstacleTrajectory)
		.def("appendObstacleStatic", &Planner::appendObstacleStatic)
		.def("getObstacleTrajectories",
			 &Planner::getObstacleTrajectories,
			 py::return_value_policy::take_ownership)
		.def("setSolverMaxTime", &Planner::setSolverMaxTime)
		.def("setSolverTimeStep", &Planner::setSolverTimeStep)
		.def("solveTrajectory", &Planner::solveTrajectory, py::return_value_policy::take_ownership)
		.def("saveToJson", &Planner::saveToJson, py::return_value_policy::take_ownership)
		.def("loadFromJson", &Planner::loadFromJson);
}
