#include "IterativeLQR.hpp"
#include "AvConversions.hpp"
#include "AvStructs.hpp"
#include "xtensor/xbuilder.hpp"
#include <fstream>
#include <iostream>

using namespace av_structs;
using namespace av_conversions;

namespace iterative_lqr
{
IterativeLQR::IterativeLQR() {}

IterativeLQR::IterativeLQR(
	AvState init, AvState goal, AvParams config, Boundary av_outline, double max_time, double dt)
	: initial_state {init}
	, goal_state {goal}
	, vehicle_config {config}
	, vehicle_outline {av_outline}
	, solver_max_time {max_time}
	, solver_dt {dt}
{}

IterativeLQR::~IterativeLQR() {}

void IterativeLQR::setGoal(AvState goal)
{
	goal_state = goal;
}

void IterativeLQR::setInitialState(AvState init)
{
	initial_state = init;
}

void IterativeLQR::setVehicleConfig(AvParams config)
{
	vehicle_config = config;
}

void IterativeLQR::setVehicleOutline(Boundary av_outline)
{
	vehicle_outline = av_outline;
}

void IterativeLQR::setSolverMaxTime(double max_time)
{
	solver_max_time = max_time;
}

void IterativeLQR::setSolverTimeStep(double dt)
{
	solver_dt = dt;
}

AvTrajectory IterativeLQR::solveTrajectory()
{
	auto state = av_state_to_xtensor(initial_state);
	auto goal = av_state_to_xtensor(goal_state);

	state.reshape({1, 5});
	goal.reshape({1, 5});

	size_t num_steps = ceil(solver_max_time / solver_dt);
	size_t state_size = AvState::SIZE;
	size_t action_size = AvAction::SIZE;

	auto steps = xt::ones<double>({num_steps, size_t(1)});

	auto X_desired = num_steps * goal;
	auto U_desired = xt::zeros<double>({num_steps, action_size});
	auto X_nominal = num_steps * state;
	auto U_nominal = xt::zeros<double>({num_steps, action_size});

	double epsilon = 0.001;

	auto Q = 100 * xt::eye<double>({state_size});
	auto Q_f = 10 * xt::eye<double>({state_size});
	auto R = 1 * xt::eye<double>({state_size});
	// Initialize the desired trajectory (a trajectory of all goal states)
	// Initialize nominal
	//
	// Define optimality epsilon
	//
	// Initial Q and R
	//
	// iterations of iLQR
	//
	//     get x_bar and u_bar desired
	//
	//     Initialize Ricatti variables S2, S1, S0
	//
	//     Linearize dynamics
	//
	//     Solve Ricatti backwards in time
	//          Get our current Ricattie Variables
	//          Linearize our system dynamics
	//          Step S2, S1, and S0
	//          Save S2, S1, and S0
	//
	//     Run forward prediction using Ricatti Solution
	//
	//          Compute linearization fo B
	//
	//          Get the Ricatti variables at this time step
	//
	//          calculate u bar star
	//
	//          convert to  u actual
	//
	//          update x actual (Euler method)
	//     Check for convergence based on epsilon
	// Return a trajectory

	// Produce a dummy solution for now.

	// Create the output trajectory.
	AvTrajectory traj;
	traj.outline = vehicle_outline;
	AvState av_state {0, 0, 0.3, -0.3, 1.0};
	traj.table.push_back(av_state);
	traj.dt = solver_dt;
	traj.av_parameters = vehicle_config;
	return traj;
}

xt::xarray<double>
IterativeLQR::dynamics(xt::xarray<double> input, double turn_rate, double accel_f)
{
	xt::xarray<double> output {
		input(AvState::VEL_F) * cos(input(AvState::DELTA_F)) * cos(input(AvState::PSI)),
		input(AvState::VEL_F) * cos(input(AvState::DELTA_F)) * cos(input(AvState::PSI)),
		input(AvState::VEL_F) * sin(input(AvState::DELTA_F)),
		turn_rate,
		accel_f};
	return (std::move(output));
}

} // namespace iterative_lqr
