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
	xt::xarray<double> Q = 100.0 * xt::eye<double>({state_size});
	xt::xarray<double> Q_f = 10.0 * xt::eye<double>({state_size});
	xt::xarray<double> R = 1.0 * xt::eye<double>({action_size});
	xt::xarray<double> inv_R = xt::linalg::inv(R);

	// Iterations of iLQR
	do
	{
		// Get x_bar and u_bar desired
		auto X_bar_desired = X_desired - X_nominal;
		auto U_bar_desired = U_desired - U_nominal;

		xt::xarray<double> X_bar_d =
			xt::expand_dims(xt::view(X_bar_desired, num_steps-1, xt::all()), 1);
		xt::xarray<double> X_bar_d_transp = xt::transpose(X_bar_d);

		std::cout << X_bar_d << std::endl;

		// Initialize Ricatti variables S2
		std::vector<size_t> shape = {num_steps, state_size, state_size};
		xt::xarray<double> S_2 = xt::zeros<double>(shape);
		auto S_2_T = xt::view(S_2, num_steps - 1, xt::all(), xt::all());
		S_2_T = Q_f;

		// Initialize Ricatti variable S1
		shape = {num_steps, state_size};
		xt::xarray<double> S_1 = xt::zeros<double>(shape);
		auto S_1_T = xt::view(S_1, num_steps - 1, xt::all());
		S_1_T = -2.0 * xt::squeeze(xt::linalg::dot(Q_f, X_bar_d));

		// Initialize Ricatti variable S0
		shape = {num_steps};
		xt::xarray<double> S_0 = xt::zeros<double>(shape);
		auto S_0_T = xt::view(S_0, num_steps - 1);
		xt::xarray<double> temp = xt::linalg::dot(X_bar_d_transp, Q_f);
		S_0_T = xt::squeeze(xt::linalg::dot(temp, X_bar_d));

		xt::xarray<double> B_t_transp {{0, 0, 0, 1, 0}, {0, 0, 0, 0, 1}};
		xt::xarray<double> B_t = xt::transpose(B_t_transp, {1, 0});
		// Solve Ricatti backwards in time
		for(size_t t = num_steps - 1; t > 0; --t)
		{
			// Get our current Ricatti Variables
			xt::xarray<double> curr_S_2 = xt::view(S_2, t, xt::all(), xt::all());
			xt::xarray<double> curr_S_1 = xt::view(S_1, t, xt::all());
			xt::xarray<double> curr_S_0 = xt::view(S_0, t);
			xt::xarray<double> curr_X_nominal = xt::view(X_nominal, t, xt::all());

			// Linearize our system dynamics
			auto A_t = jacobian(curr_X_nominal);

			std::cout << "A" << std::endl;
			std::cout<< A_t << std::endl;

			// Step S2
			auto S_2_dot = Q;
			S_2_dot = S_2_dot + xt::linalg::dot(curr_S_2, A_t);

			std::cout << S_2_dot << std::endl;
			S_2_dot = S_2_dot + xt::linalg::dot(xt::transpose(A_t), curr_S_2);

			std::cout << S_2_dot << std::endl;
			auto temp = xt::linalg::dot(curr_S_2, B_t);

			std::cout << S_2_dot << std::endl;
			temp = xt::linalg::dot(temp, inv_R);

			std::cout << S_2_dot << std::endl;
			temp = xt::linalg::dot(temp, B_t_transp);

			std::cout << S_2_dot << std::endl;
			S_2_dot = S_2_dot - xt::linalg::dot(temp, curr_S_2);

			std::cout << S_2_dot << std::endl;


			// Step S1
			
			// Step S0

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
			xt::xarray<double> current_S_2 = xt::view(S_2, t, xt::all(), xt::all());
			xt::xarray<double> current_S_1 = xt::view(S_1, t, xt::all());
			current_S_1.reshape({state_size, size_t(1)});
			xt::xarray<double> current_S_0 = xt::view(S_0, t);
			xt::xarray<double> U_bar_d = xt::view(U_bar_desired, t, xt::all());
			xt::xarray<double> X_act = xt::view(X_actual, size_t(t - 1), xt::all());
			xt::xarray<double> X_nom = xt::view(X_nominal, t, xt::all());
			xt::xarray<double> U_nom = xt::view(U_nominal, t, xt::all());
			X_nom.reshape({size_t(1), state_size});

			// Calculate U Bar Star
			xt::xarray<double> temp1 = xt::linalg::dot(inv_R, B_t_transp);
			xt::xarray<double> temp2 = X_act - X_nom + 0.5 * current_S_1;
			xt::xarray<double> temp3 = xt::transpose(temp2, {1, 0});
			std::cout << "Worked!" << std::endl;
			xt::xarray<double> temp4 = xt::linalg::dot(current_S_2, temp3);
			xt::xarray<double> temp5 = xt::linalg::dot(temp1, temp4);

			xt::xarray<double> U_bar_star = U_bar_d - temp5;
			std::cout << "Worked!" << std::endl;
			auto U_act = xt::view(U_actual, t, xt::all());
			std::cout << "Worked!" << std::endl;
			U_act = U_nom + U_bar_star;

			std::cout << "Worked!" << std::endl;
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
