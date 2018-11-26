#ifndef AVTRAJECTORYPLANNER_HPP
#define AVTRAJECTORYPLANNER_HPP

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

	~Planner();
};
} // namespace av_trajectory_planner

#endif
