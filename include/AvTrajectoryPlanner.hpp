#ifndef AVTRAJECTORYPLANNER_HPP
#define AVTRAJECTORYPLANNER_HPP

#include "AvStructs.hpp"
#include <jsoncpp/json/json.h>
#include <math.h>
#include <vector>

namespace av_trajectory_planner
{

///
/// The class for handling trajectory optimization.
///
class Planner
{
public:
	Planner();

	Planner(av_structs::AvState init,
			av_structs::AvState goal,
			av_structs::AvParams config,
			av_structs::Boundary av_outline,
			double max_time = 5.0,
			double dt = 0.01);

	~Planner();

	void setGoal(av_structs::AvState goal);

	void setInitialState(av_structs::AvState init);

	void setVehicleConfig(av_structs::AvParams config);

	void setVehicleOutline(av_structs::Boundary av_outline);

	void clearObstacles();

	void appendObstacleTrajectory(av_structs::ObstacleTrajectory obstacle);

	void appendObstacleStatic(av_structs::ObstacleStatic obstacle);

	std::vector<av_structs::ObstacleTrajectory> getObstacleTrajectories();

	void setSolverMaxTime(double max_time);

	void setSolverTimeStep(double dt);

	void loadFromJson(std::string raw_json);

	std::string saveToJson();

	av_structs::AvTrajectory solveTrajectory();

private:
	av_structs::AvState dynamics(av_structs::AvState input, double turn_rate, double accel_f);

	av_structs::AvState
	apply_dynamics(av_structs::AvState input, av_structs::AvState current_dynamics, double);

	av_structs::AvState euler_step_unforced(av_structs::AvState input, double dt);

	av_structs::AvState initial_state;

	av_structs::AvState goal_state;

	av_structs::AvParams vehicle_config;

	av_structs::Boundary vehicle_outline;

	std::vector<av_structs::ObstacleTrajectory> obstacles;

	double solver_max_time;

	double solver_dt;
};
} // namespace av_trajectory_planner

#endif
