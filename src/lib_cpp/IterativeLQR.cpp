#include "IterativeLQR.hpp"
#include "AvConversions.hpp"
#include "AvStructs.hpp"
#include "xtensor-blas/xblas.hpp"
#include "xtensor-blas/xlinalg.hpp"
#include <fstream>
#include <iostream>
#include <vector>

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
	// Initialize the desired trajectory (a trajectory of all goal states)
	auto state = av_state_to_xtensor(initial_state);
	auto goal = av_state_to_xtensor(goal_state);

	state.reshape({1, 5});
	goal.reshape({1, 5});

	size_t num_steps = ceil(solver_max_time / solver_dt);
	size_t state_size = AvState::SIZE;
	size_t action_size = AvAction::SIZE;

	auto steps = xt::ones<double>({num_steps, size_t(1)});

	auto X_desired = steps * goal;
	auto U_desired = xt::zeros<double>({num_steps, action_size});

	// Initialize nominal
	auto X_nominal = steps * state;
	auto U_nominal = xt::zeros<double>({num_steps, action_size});
	// Define optimality epsilon
	double epsilon = 0.001;

	// Initial Q and R
	auto Q = 100.0 * xt::eye<double>({state_size});
	auto Q_f = 10.0 * xt::eye<double>({state_size});
	auto R = 1.0 * xt::eye<double>({state_size});

	// Iterations of iLQR
	do
	{
		// Get x_bar and u_bar desired
		auto X_bar_desired = X_desired - X_nominal;
		auto U_bar_desired = U_desired - U_nominal;


		// Initialize Ricatti variables S2, S1, S0
		xt::xarray<double> S_2_zeros =
			xt::zeros<double>({size_t(num_steps - 1), state_size, state_size});
		auto S_2 = xt::stack(xtuple(S_2_zeros, Q_f), 1);

		xt::xarray<double> S_1_zeros = xt::zeros<double>({size_t(num_steps - 1), state_size});
		xt::xarray<double> X_bar_d = xt::expand_dims(xt::view(X_bar_desired, num_steps, xt::all()), 0);

		xt::xarray<double> X_bar_d_transp = xt::transpose(X_bar_d, {1, 0});
		
		std::cout << Q_f << std::endl;
		std::cout << X_bar_d << std::endl;
		xt::xarray<double> S_1_T = -2.0 * xt::linalg::dot(Q_f, X_bar_d_transp);
		xt::xarray<double> S_1 = xt::stack(xtuple(S_1_zeros, S_1_T), 1);

		xt::xarray<double> S_0_zeros = xt::zeros<double>({num_steps - 1, size_t(1)});
		xt::xarray<double> S_0_T_left = xt::linalg::dot(X_bar_d, Q_f);
		xt::xarray<double> S_0_T = xt::linalg::dot(S_0_T_left, X_bar_d_transp);
		xt::xarray<double> S_0 = xt::stack(xtuple(S_0_zeros, S_0_T), 1);

		// std::cout << S_2 << std::endl;
		// std::cout << S_0 << std::endl;
		// std::cout << S_1 << std::endl;

		// Solve Ricatti backwards in time
		for(size_t t = num_steps; t > 0; --t)
		{
			// Get our current Ricatti Variables
			xt::xarray<double> current_S_2 = xt::view(S_2, t, xt::all(), xt::all());
			xt::xarray<double> current_S_1 = xt::view(S_1, t, xt::all());
			xt::xarray<double> current_S_0 = xt::view(S_0, t, size_t(1));
			xt::xarray<double> current_X_nominal = xt::view(X_nominal, t, xt::all());

			// Linearize our system dynamics
			auto A_t = jacobian(current_X_nominal);
			xt::xarray<double> B_t {0, 0, 0, 1, 1};
			B_t.reshape({5, 1});

			// Step S2, S1, and S0
			auto B_t_transpose = xt::transpose(B_t, {1, 0});

			auto inv_R = xt::linalg::inv(R);

			// auto temp2 = xt::linalg::dot(current_S_2, current_S_2);
			// auto temp2 = xt::linalg::dot(inv_R, B_t_transpose);

			// auto temp3 = xt::linalg::dot(temp1, temp2);
			// auto temp4 = xt::linalg::dot(temp3, current_S_2);
			// auto next_S_2 = current_S_2 + solver_dt * (Q - temp4);
		}

		break;
	} while(true);
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

xt::xarray<double> IterativeLQR::jacobian(xt::xarray<double> input)
{
	xt::xarray<double> output {
		{0,
		 0,
		 input(AvState::VEL_F) * cos(input(AvState::DELTA_F)) * sin(input(AvState::PSI)),
		 input(AvState::VEL_F) * sin(input(AvState::DELTA_F)) * cos(input(AvState::PSI)),
		 cos(input(AvState::DELTA_F)) * cos(input(AvState::PSI))},
		{0,
		 0,
		 -input(AvState::VEL_F) * cos(input(AvState::DELTA_F)) * cos(input(AvState::PSI)),
		 input(AvState::VEL_F) * sin(input(AvState::DELTA_F)) * sin(input(AvState::PSI)),
		 input(AvState::VEL_F) * cos(input(AvState::DELTA_F)) * sin(input(AvState::PSI))},
		{0,
		 0,
		 0,
		 -input(AvState::VEL_F) * cos(input(AvState::DELTA_F)),
		 sin(input(AvState::DELTA_F))},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0}};
	return (std::move(output));
}

xt::xarray<double>
IterativeLQR::dynamics(xt::xarray<double> input, double turn_rate, double accel_f)
{
	xt::xarray<double> output {
		input(AvState::VEL_F) * cos(input(AvState::DELTA_F)) * cos(input(AvState::PSI)),
		input(AvState::VEL_F) * cos(input(AvState::DELTA_F)) * sin(input(AvState::PSI)),
		input(AvState::VEL_F) * sin(input(AvState::DELTA_F)),
		turn_rate,
		accel_f};
	return (std::move(output));
}

} // namespace iterative_lqr
