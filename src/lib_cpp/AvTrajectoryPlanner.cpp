#include <fstream>
#include <iostream>

#include "AvTrajectoryPlanner.hpp"

namespace av_trajectory_planner
{
Planner::Planner() {}

Planner::Planner(
	AvState init, AvState goal, AvParams config, Boundary av_outline, double max_time, double dt)
	: initial_state {init}
	, goal_state {goal}
	, vehicle_config {config}
	, vehicle_outline {av_outline}
	, solver_max_time {max_time}
	, solver_dt {dt}
{}

Planner::~Planner() {}

void Planner::setGoal(AvState goal)
{
	goal_state = goal;
}

void Planner::setInitialState(AvState init)
{
	initial_state = init;
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
	static_traj.obs_outline = obs_static.obs_outline;
	static_traj.pose_table.push_back(obs_static.obs_pose);
	static_traj.pose_table.push_back(obs_static.obs_pose);
	static_traj.dt = solver_max_time;
	obstacles.push_back(std::move(static_traj));
}

std::vector<ObstacleTrajectory> Planner::getObstacleTrajectories()
{
	return obstacles;
}

void Planner::setSolverMaxTime(double max_time)
{
	solver_max_time = max_time;
}

void Planner::setSolverTimeStep(double dt)
{
	solver_dt = dt;
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
	solver_max_time = root.get("solver_max_time", 5.0).asDouble();
	solver_dt = root.get("solver_dt", 0.01).asDouble();
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
	root["solver_max_time"] = solver_max_time;
	root["solver_dt"] = solver_dt;

	Json::StyledWriter writer;
	return writer.write(root);
}

AvTrajectory Planner::solveTrajectory()
{
	// Produce a dummy solution for now.
	AvTrajectory traj;
	traj.av_outline = vehicle_outline;
	AvState av_state {0, 0, 0.3, 0, 0.1};
	traj.av_state_table.push_back(av_state);
	double dt = 0.5;
	for(int i = 0; i < 50; i++)
	{
		av_state = euler_step_unforced(av_state, dt);
		traj.av_state_table.push_back(av_state);
	}
	traj.dt = dt;
	traj.av_parameters = vehicle_config;
	return traj;
}

AvState Planner::dynamics(AvState input, double turn_rate, double accel_f)
{
	AvState output;
	output.x = input.vel_f * cos(input.delta_f) * cos(input.psi);
	output.y = input.vel_f * cos(input.delta_f) * sin(input.psi);
	output.psi = input.vel_f * sin(input.delta_f);
	output.delta_f = turn_rate;
	output.vel_f = accel_f;
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
	AvState current_dynamics = dynamics(input, 0.0, 0.0);
	return (std::move(apply_dynamics(input, current_dynamics, dt)));
}

} // namespace av_trajectory_planner
