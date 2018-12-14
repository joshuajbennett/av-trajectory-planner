#include <fstream>
#include <iostream>

#include "AvTrajectoryPlanner.hpp"
#include "IterativeLQR.hpp"

using namespace av_structs;
using namespace iterative_lqr;

namespace av_trajectory_planner
{
Planner::Planner() {}

Planner::Planner(
	AvState init, AvState goal, AvParams config, Boundary av_outline, SolverParams solver_settings)
	: initial_state {init}
	, goal_state {goal}
	, vehicle_config {config}
	, vehicle_outline {av_outline}
	, settings {solver_settings}
{
	if(settings.constrain_velocity)
	{
		std::cout << "Velocity clamping enabled." << std::endl;
	}
	if(settings.constrain_steering_angle)
	{
		std::cout << "Steering angle clamping enabled." << std::endl;
	}
}

Planner::~Planner() {}

void Planner::setGoal(AvState goal)
{
	goal_state = goal;
}

AvState Planner::getGoal()
{
	return goal_state;
}

void Planner::setInitialState(AvState init)
{
	initial_state = init;
}

AvState Planner::getInitialState()
{
	return initial_state;
}

void Planner::setVehicleConfig(AvParams config)
{
	vehicle_config = config;
}

void Planner::setVehicleOutline(Boundary av_outline)
{
	vehicle_outline = av_outline;
}

void Planner::clearObstacles()
{
	obstacles.resize(0);
}

void Planner::appendObstacleTrajectory(ObstacleTrajectory obs_trajectory)
{
	obstacles.push_back(std::move(obs_trajectory));
}

void Planner::appendObstacleStatic(ObstacleStatic obs_static)
{
	ObstacleTrajectory static_traj;
	static_traj.outline = obs_static.outline;
	static_traj.table.push_back(obs_static.obs_pose);
	static_traj.table.push_back(obs_static.obs_pose);
	static_traj.dt = settings.max_time;
	obstacles.push_back(std::move(static_traj));
}

std::vector<ObstacleTrajectory> Planner::getObstacleTrajectories()
{
	return obstacles;
}

void Planner::setSolverMaxTime(double max_time)
{
	settings.max_time = max_time;
}

void Planner::setSolverTimeStep(double dt)
{
	settings.solver_dt = dt;
}

void Planner::setSolverEpsilon(double epsilon)
{
	settings.epsilon_suboptimality = epsilon;
}

void Planner::setSolverMaxIterations(unsigned int max_iter)
{
	settings.max_iterations = max_iter;
}

void Planner::enableVelocityConstraint()
{
	settings.constrain_velocity = true;
}

void Planner::disableVelocityConstraint()
{
	settings.constrain_velocity = false;
}

void Planner::enableSteeringConstraint()
{
	settings.constrain_steering_angle = true;
}

void Planner::disableSteeringConstraint()
{
	settings.constrain_steering_angle = false;
}

void Planner::loadFromJson(std::string raw_json)
{
	Json::Value root;
	Json::Reader reader;
	bool parsingSuccessful = reader.parse(raw_json, root);
	if(!parsingSuccessful)
	{
		// report to the user the failure and their locations in the document.
		std::cout << "Failed to parse configuration\n" << reader.getFormattedErrorMessages();
		return;
	}
	initial_state.loadFromJson(root["initial_state"]);
	goal_state.loadFromJson(root["goal_state"]);
	vehicle_config.loadFromJson(root["vehicle_config"]);
	vehicle_outline.loadFromJson(root["vehicle_outline"]);
	obstacles.resize(0);
	for(auto obstacle : root["obstacles"])
	{
		ObstacleTrajectory temp_obstacle;
		temp_obstacle.loadFromJson(obstacle);
		obstacles.push_back(temp_obstacle);
	}
	settings.loadFromJson(root["settings"]);
}

std::string Planner::saveToJson()
{
	Json::Value root;
	root["initial_state"] = initial_state.saveToJson();
	root["goal_state"] = goal_state.saveToJson();
	root["vehicle_config"] = vehicle_config.saveToJson();
	root["vehicle_outline"] = vehicle_outline.saveToJson();
	Json::Value obstacle_list;
	for(auto obstacle : obstacles)
	{
		obstacle_list.append(obstacle.saveToJson());
	}
	root["obstacles"] = obstacle_list;
	root["settings"] = settings.saveToJson();
	Json::StyledWriter writer;
	return writer.write(root);
}

AvTrajectory Planner::solveTrajectory()
{
	IterativeLQR ilqr =
		IterativeLQR(initial_state, goal_state, vehicle_config, vehicle_outline, settings);

	return std::move(ilqr.solveTrajectory());
}

AvState Planner::dynamics(AvState input, AvAction action)
{
	AvState output;
	output.x = input.vel_f * cos(input.delta_f) * cos(input.psi);
	output.y = input.vel_f * cos(input.delta_f) * sin(input.psi);
	output.psi = input.vel_f * sin(input.delta_f);
	output.delta_f = action.turn_rate;
	output.vel_f = action.accel_f;
	return (std::move(output));
}

AvState Planner::apply_dynamics(AvState input, AvState current_dynamics, double dt)
{
	AvState output;
	output = input + current_dynamics * dt;
	return (std::move(output));
}

AvState Planner::euler_step_unforced(AvState input, double dt)
{
	AvState current_dynamics = dynamics(input, AvAction {0.0, 0.0});
	return (std::move(apply_dynamics(input, current_dynamics, dt)));
}

} // namespace av_trajectory_planner
