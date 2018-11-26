#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION

#include "AvTrajectoryPlanner.hpp"
#include <iostream>
#include <numpy/ndarrayobject.h>
#include <numpy/npy_math.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace av_trajectory_planner
{

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
	py::class_<PlannerPython>(m, "Planner").def(py::init<>());
}
