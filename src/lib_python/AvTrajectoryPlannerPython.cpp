#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include "AvTrajectoryPlanner.hpp"
#include <iostream>
#include <numpy/ndarrayobject.h>
#include <numpy/npy_math.h>
#include <pybind11/pybind11.h>

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
	~PlannerPython() {}

private:
	Planner planner;
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
		.def_readwrite("l_f", &AvParams::l_f);
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
		.def_readwrite("dt", &ObstacleTrajectory::dt);
	py::class_<AvTrajectory>(m, "AvTrajectory")
		.def(py::init<>())
		.def_readwrite("av_outline", &AvTrajectory::av_outline)
		.def_readwrite("av_state_table", &AvTrajectory::av_state_table)
		.def_readwrite("dt", &AvTrajectory::dt)
		.def_readwrite("av_parameters", &AvTrajectory::av_parameters);
	py::class_<PlannerPython>(m, "Planner").def(py::init<>());
}
