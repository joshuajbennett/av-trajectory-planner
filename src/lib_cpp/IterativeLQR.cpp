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

	xt::xarray<double> steps = xt::ones<double>({num_steps, size_t(1)});

	xt::xarray<double> X_desired = steps * goal;
	xt::xarray<double> U_desired = xt::zeros<double>({num_steps, action_size});

	// Initialize nominal
	xt::xarray<double> X_nominal = steps * state;
	xt::xarray<double> U_nominal = xt::zeros<double>({num_steps, action_size});
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
		xt::xarray<double> S_2_T = Q_f;
		S_2_T.reshape({size_t(1), state_size, state_size});
		auto S_2 = xt::concatenate(xtuple(S_2_zeros, S_2_T), 0);

		xt::xarray<double> S_1_zeros = xt::zeros<double>({size_t(num_steps - 1), state_size});
		xt::xarray<double> X_bar_d =
			xt::expand_dims(xt::view(X_bar_desired, num_steps, xt::all()), 0);

		xt::xarray<double> X_bar_d_transp = xt::transpose(X_bar_d, {1, 0});

		std::cout << Q_f << std::endl;
		std::cout << X_bar_d << std::endl;
		xt::xarray<double> S_1_T = -2.0 * xt::linalg::dot(Q_f, X_bar_d_transp);
		xt::xarray<double> S_1 = xt::concatenate(xtuple(S_1_zeros, S_1_T), 0);

		xt::xarray<double> S_0_zeros = xt::zeros<double>({num_steps - 1, size_t(1)});
		xt::xarray<double> S_0_T_left = xt::linalg::dot(X_bar_d, Q_f);
		xt::xarray<double> S_0_T = xt::linalg::dot(S_0_T_left, X_bar_d_transp);
		xt::xarray<double> S_0 = xt::concatenate(xtuple(S_0_zeros, S_0_T), 0);

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

			//auto inv_R = xt::linalg::inv(R);

			// auto temp2 = xt::linalg::dot(current_S_2, current_S_2);
			// auto temp2 = xt::linalg::dot(inv_R, B_t_transpose);

			// auto temp3 = xt::linalg::dot(temp1, temp2);
			// auto temp4 = xt::linalg::dot(temp3, current_S_2);
			// auto next_S_2 = current_S_2 + solver_dt * (Q - temp4);
		}

		xt::xarray<double> X_actual = steps * state;
		xt::xarray<double> U_actual = xt::zeros<double>({num_steps, action_size});
		xt::xarray<double> inv_R = xt::linalg::inv(R);
		for(size_t t = 1; t < num_steps; ++t)
		{
			xt::xarray<double> B_t_transp {{0, 0, 0, 1, 1}};
			xt::xarray<double> B_t = B_t_transp;
			B_t.reshape({5, 1});
			xt::xarray<double> current_S_2 = xt::view(S_2, t, xt::all(), xt::all());
			xt::xarray<double> current_S_1 = xt::view(S_1, t, xt::all());
			current_S_1.reshape({5, 1});
			xt::xarray<double> current_S_0 = xt::view(S_0, t, size_t(1));

			xt::xarray<double> U_bar_d = xt::view(U_bar_desired, t, xt::all());
			xt::xarray<double> X_act = xt::view(X_actual, size_t(t - 1), xt::all());
			xt::xarray<double> X_nom = xt::view(X_nominal, t, xt::all());
			xt::xarray<double> U_nom = xt::view(U_nominal, t, xt::all());
			X_nom.reshape({1, 5});

			// Calculate U Bar Star
			xt::xarray<double> temp1 = xt::linalg::dot(inv_R, B_t);
			xt::xarray<double> temp2 = X_act - X_nom;
			xt::xarray<double> temp3 = xt::transpose(temp2, {1, 0});
			xt::xarray<double> temp4 = xt::linalg::dot(current_S_2, temp3);
			xt::xarray<double> temp5 = xt::linalg::dot(temp2, temp4);
			xt::xarray<double> U_bar_star = U_bar_d - temp5 + 0.5 * current_S_1;
			auto U_act = xt::view(U_actual, t, xt::all());
			U_act = xt::squeeze(U_nom + U_bar_star);
			xt::view(U_actual, t, xt::all()) = U_act;

			// Apply forward prediction
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

xt::xarray<double> IterativeLQR::jacobian(xt::xarray<double> state)
{
	xt::xarray<double> output {
		{0,
		 0,
		 state(AvState::VEL_F) * cos(state(AvState::DELTA_F)) * sin(state(AvState::PSI)),
		 state(AvState::VEL_F) * sin(state(AvState::DELTA_F)) * cos(state(AvState::PSI)),
		 cos(state(AvState::DELTA_F)) * cos(state(AvState::PSI))},
		{0,
		 0,
		 -state(AvState::VEL_F) * cos(state(AvState::DELTA_F)) * cos(state(AvState::PSI)),
		 state(AvState::VEL_F) * sin(state(AvState::DELTA_F)) * sin(state(AvState::PSI)),
		 state(AvState::VEL_F) * cos(state(AvState::DELTA_F)) * sin(state(AvState::PSI))},
		{0,
		 0,
		 0,
		 -state(AvState::VEL_F) * cos(state(AvState::DELTA_F)),
		 sin(state(AvState::DELTA_F))},
		{0, 0, 0, 0, 0},
		{0, 0, 0, 0, 0}};
	return (std::move(output));
}

xt::xarray<double> IterativeLQR::dynamics(xt::xarray<double> state, xt::xarray<double> input)
{
	xt::xarray<double> output {
		state(AvState::VEL_F) * cos(state(AvState::DELTA_F)) * cos(state(AvState::PSI)),
		state(AvState::VEL_F) * cos(state(AvState::DELTA_F)) * sin(state(AvState::PSI)),
		state(AvState::VEL_F) * sin(state(AvState::DELTA_F)),
		input(AvAction::TURN_RATE),
		input(AvAction::ACCEL_F)};
	return (std::move(output));
}

} // namespace iterative_lqr
