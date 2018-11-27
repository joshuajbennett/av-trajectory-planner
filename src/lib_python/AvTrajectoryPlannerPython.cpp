#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include "AvTrajectoryPlanner.hpp"
#include <iostream>
#include <numpy/ndarrayobject.h>
#include <numpy/npy_math.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

namespace av_trajectory_planner
{

#if PY_MAJOR_VERSION >= 3
void* init_numpy()
{
	import_array();
	return nullptr;
}
#else
void init_numpy()
{
	import_array();
}
#endif

class PlannerPython
{
public:
	PlannerPython() {}

	void setGoal(AvState goal)
	{
		planner.setGoal(goal);
	}

	void setInitialState(AvState init)
	{
		planner.setInitialState(init);
	}

	void setVehicleConfig(AvParams config)
	{
		planner.setVehicleConfig(config);
	}

	void setVehicleOutline(Boundary av_outline)
	{
		planner.setVehicleOutline(av_outline);
	}

	void clearObstacles()
	{
		planner.clearObstacles();
	}

	void appendObstacleTrajectory(ObstacleTrajectory obstacle)
	{
		planner.appendObstacleTrajectory(obstacle);
	}

	void appendObstacleStatic(ObstacleStatic obstacle)
	{
		planner.appendObstacleStatic(obstacle);
	}

	void setSolverMaxTime(double max_time)
	{
		planner.setSolverMaxTime(max_time);
	}

	void setSolverTimeStep(double dt)
	{
		planner.setSolverTimeStep(dt);
	}

	AvTrajectory solveTrajectory()
	{
		return planner.solveTrajectory();
	}

private:
	Planner planner;

	py::object matToNumpyArray(int dims, npy_intp* shape, int type, void* data)
	{
		PyObject* pyArray = PyArray_SimpleNewFromData(dims, shape, type, data);
		/* This line makes a copy: */
		PyObject* pyArrayCopied =
			PyArray_FROM_OTF(pyArray, type, NPY_ARRAY_ENSURECOPY | NPY_ARRAY_ENSUREARRAY);
		/* And this line gets rid of the old object which caused a memory leak: */
		Py_DECREF(pyArray);

		py::handle numpyArrayHandle = py::handle(pyArrayCopied);
		py::object numpyArray = py::reinterpret_steal<py::object>(numpyArrayHandle);

		return numpyArray;
	}
};
} // namespace av_trajectory_planner

using namespace av_trajectory_planner;

PYBIND11_MODULE(AvTrajectoryPlanner, m)
{
	py::class_<AvState>(m, "AvState")
		.def(py::init<>())
		.def_readwrite("x", &AvState::x)
		.def_readwrite("y", &AvState::y)
		.def_readwrite("psi", &AvState::psi)
		.def_readwrite("delta_f", &AvState::delta_f)
		.def_readwrite("vel_f", &AvState::vel_f);
	py::class_<AvParams>(m, "AvParams")
		.def(py::init<>())
		.def_readwrite("l_r", &AvParams::l_r)
		.def_readwrite("l_f", &AvParams::l_r)
		.def_readwrite("max_delta_f", &AvParams::max_delta_f)
		.def_readwrite("max_accel_f", &AvParams::max_accel_f);
	py::class_<Point>(m, "Point")
		.def(py::init<>())
		.def_readwrite("x", &Point::x)
		.def_readwrite("y", &Point::y);
	py::class_<Boundary>(m, "Boundary")
		.def(py::init<>())
		.def_readwrite("vertices", &Boundary::vertices);
	py::class_<Pose>(m, "Pose")
		.def(py::init<>())
		.def_readwrite("x", &Pose::x)
		.def_readwrite("y", &Pose::y)
		.def_readwrite("theta", &Pose::theta);
	py::class_<ObstacleTrajectory>(m, "ObstacleTrajectory")
		.def(py::init<>())
		.def_readwrite("obs_outline", &ObstacleTrajectory::obs_outline)
		.def_readwrite("pose_table", &ObstacleTrajectory::pose_table)
		.def_readwrite("dt", &ObstacleTrajectory::dt)
		.def("interpolate", &ObstacleTrajectory::interpolate);
	py::class_<ObstacleStatic>(m, "ObstacleStatic")
		.def(py::init<>())
		.def_readwrite("obs_outline", &ObstacleStatic::obs_outline)
		.def_readwrite("obs_pose", &ObstacleStatic::obs_pose);
	py::class_<AvTrajectory>(m, "AvTrajectory")
		.def(py::init<>())
		.def_readwrite("av_outline", &AvTrajectory::av_outline)
		.def_readwrite("av_state_table", &AvTrajectory::av_state_table)
		.def_readwrite("dt", &AvTrajectory::dt)
		.def_readwrite("av_parameters", &AvTrajectory::av_parameters)
		.def("interpolate", &AvTrajectory::interpolate);
	py::class_<PlannerPython>(m, "Planner")
		.def(py::init<>())
		.def("setGoal", &PlannerPython::setGoal)
		.def("setInitialState", &PlannerPython::setInitialState)
		.def("clearObstacles", &PlannerPython::clearObstacles)
		.def("appendObstacleTrajectory", &PlannerPython::appendObstacleTrajectory)
		.def("appendObstacleStatic", &PlannerPython::appendObstacleStatic)
		.def("setSolverMaxTime", &PlannerPython::setSolverMaxTime)
		.def("setSolverTimeStep", &PlannerPython::setSolverTimeStep)
		.def("solveTrajectory",
			 &PlannerPython::solveTrajectory,
			 py::return_value_policy::take_ownership);
}
