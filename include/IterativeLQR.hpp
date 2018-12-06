#ifndef ITERATIVELQR_HPP
#define ITERATIVELQR_HPP

#include "AvStructs.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xbuilder.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xreducer.hpp"
#include "xtensor/xtensor.hpp"
#include "xtensor/xview.hpp"

namespace iterative_lqr
{

///
/// The class for handling trajectory optimization.
///
class IterativeLQR
{
public:
	IterativeLQR();

	IterativeLQR(av_structs::AvState init,
				 av_structs::AvState goal,
				 av_structs::AvParams config,
				 av_structs::Boundary av_outline,
				 av_structs::SolverParams solver_settings);

	~IterativeLQR();

	void setGoal(av_structs::AvState goal);

	void setInitialState(av_structs::AvState init);

	void setVehicleConfig(av_structs::AvParams config);

	void setVehicleOutline(av_structs::Boundary av_outline);

	void setSolverMaxTime(double max_time);

	void setSolverTimeStep(double dt);

	av_structs::AvTrajectory solveTrajectory();

private:
	xt::xarray<double> dynamics(xt::xarray<double> state, xt::xarray<double> input);

	xt::xarray<double> linearized_dynamics(xt::xarray<double> state);

	double sigmoid(double x);

	av_structs::AvState initial_state;

	av_structs::AvState goal_state;

	av_structs::AvParams vehicle_config;

	av_structs::Boundary vehicle_outline;

	av_structs::SolverParams settings;
};
} // namespace iterative_lqr

#endif
