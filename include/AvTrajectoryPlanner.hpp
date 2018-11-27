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

	AvState operator*(float val)
	{
		return std::move(AvState {x * val, y * val, psi * val, delta_f * val, vel_f * val});
	}

	AvState operator+(AvState state)
	{
		return std::move(AvState {x + state.x,
								  y + state.y,
								  psi + state.psi,
								  delta_f + state.delta_f,
								  vel_f + state.vel_f});
	}
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

	Pose operator*(float val)
	{
		return std::move(Pose {x * val, y * val, theta * val});
	}

	Pose operator+(Pose pose)
	{
		return std::move(Pose {x + pose.x, y + pose.y, theta + pose.theta});
	}
};

///
/// An obstacle trajectory.
///
struct ObstacleTrajectory
{
	Boundary obs_outline;
	std::vector<Pose> pose_table;
	double dt;

	Pose interpolate(double time)
	{
		if(time < 0)
		{
			return std::move(pose_table[0]);
		}
		else if(time >= (pose_table.size() - 1) * dt)
		{
			return std::move(pose_table[pose_table.size() - 1]);
		}
		else
		{
			int first_pos = std::floor(time / dt);
			Pose first = pose_table[first_pos];
			Pose second = pose_table[first_pos + 1];
			double dist = time / dt - first_pos;
			double inv_dist = 1.0 - dist;
			return std::move(first * inv_dist + second * dist);
		}
	}
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

	AvState interpolate(double time)
	{
		if(time < 0)
		{
			return std::move(av_state_table[0]);
		}
		else if(time >= (av_state_table.size() - 1) * dt)
		{
			return std::move(av_state_table[av_state_table.size() - 1]);
		}
		else
		{
			int first_pos = std::floor(time / dt);
			AvState first = av_state_table[first_pos];
			AvState second = av_state_table[first_pos + 1];
			double dist = time / dt - first_pos;
			double inv_dist = 1.0 - dist;
			return std::move(first * inv_dist + second * dist);
		}
	}
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

	void setVehicleOutline(Boundary av_outline);

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
