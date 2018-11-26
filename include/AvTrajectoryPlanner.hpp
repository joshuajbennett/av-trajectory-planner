#ifndef AVTRAJECTORYPLANNER_HPP
#define AVTRAJECTORYPLANNER_HPP

#include <math.h>
#include <vector>
namespace av_trajectory_planner
{

///
/// Contains all state variables needed to define the state of an AV.
///
struct AvState
{
	double x;
	double y;
	double psi;
	double delta_f;
	double vel_f;
};

///
/// Contains all parameters needed to define an AV state space model.
///
struct AvParams
{
	double l_r;
	double l_f;
	double max_delta_f;
	double max_accel_f;
};

///
/// A 2d point.
///
struct Point
{
	double x;
	double y;
};

///
/// An outline of points defining a polygon boundary.
///
struct Boundary
{
	std::vector<Point> vertices;
};

///
/// A 2d pose.
///
struct Pose
{
	double x;
	double y;
	double theta;
};

///
/// An obstacle trajectory.
///
struct ObstacleTrajectory
{
	Boundary obs_outline;
	std::vector<Pose> pose_table;
	double dt;
};

///
/// A static obstacle.
///
struct ObstacleStatic
{
	Boundary obs_outline;
	Pose obs_pose;
};

///
/// An AV trajectory.
///
struct AvTrajectory
{
	Boundary av_outline;
	std::vector<AvState> av_state_table;
	double dt;
	AvParams av_parameters;
};

///
/// The class for handling trajectory optimization.
///
class Planner
{
	friend class PlannerPython;

public:
	Planner();

	Planner(AvState init,
			AvState goal,
			AvParams config,
			Boundary av_outline,
			double max_time = 5.0,
			double dt = 0.01);

	~Planner();

	void setGoal(AvState goal);

	void setInitialState(AvState init);

	void setVehicleConfig(AvParams config);

	void clearObstacles();

	void appendObstacleTrajectory(ObstacleTrajectory obstacle);

	void appendObstacleStatic(ObstacleStatic obstacle);

	void setSolverMaxTime(double max_time);

	void setSolverTimeStep(double dt);

	AvTrajectory solveTrajectory();

private:
	AvState dynamics(AvState, double, double);

	AvState apply_dynamics(AvState, AvState, double);

	AvState euler_step_unforced(AvState input, double dt);

	AvState initial_state;

	AvState goal_state;

	AvParams vehicle_config;

	Boundary vehicle_outline;

	std::vector<ObstacleTrajectory> obstacles;

	double solver_max_time;

	double solver_dt;
};
} // namespace av_trajectory_planner

#endif
