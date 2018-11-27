#ifndef AVTRAJECTORYPLANNER_HPP
#define AVTRAJECTORYPLANNER_HPP

#include <jsoncpp/json/json.h>
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

	void loadFromJson(Json::Value json)
	{
		x = json.get("x", 0.0).asDouble();
		y = json.get("y", 0.0).asDouble();
		psi = json.get("psi", 0.0).asDouble();
		delta_f = json.get("delta_f", 0.0).asDouble();
		vel_f = json.get("vel_f", 0.0).asDouble();
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		json["x"] = x;
		json["y"] = y;
		json["psi"] = psi;
		json["delta_f"] = delta_f;
		json["vel_f"] = vel_f;
		return json;
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

	void loadFromJson(Json::Value json)
	{
		l_r = json.get("l_r", 0.0).asDouble();
		l_f = json.get("l_f", 0.0).asDouble();
		max_delta_f = json.get("max_delta_f", 0.0).asDouble();
		max_accel_f = json.get("max_accel_f", 0.0).asDouble();
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		json["l_r"] = l_r;
		json["l_f"] = l_f;
		json["max_delta_f"] = max_delta_f;
		json["max_accel_f"] = max_accel_f;
		return json;
	}
};

///
/// A 2d point.
///
struct Point
{
	double x;
	double y;

	void loadFromJson(Json::Value json)
	{
		x = json.get("x", 0.0).asDouble();
		y = json.get("y", 0.0).asDouble();
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		json["x"] = x;
		json["y"] = y;
		return json;
	}
};

///
/// An outline of points defining a polygon boundary.
///
struct Boundary
{
	std::vector<Point> vertices;

	void loadFromJson(Json::Value json)
	{
		vertices.resize(0);
		for(auto point : json["vertices"])
		{
			Point temp_point;
			temp_point.loadFromJson(point);
			vertices.push_back(temp_point);
		}
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		Json::Value point_list;
		for(auto point : vertices)
		{
			point_list.append(point.saveToJson());
		}
		return json;
	}
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

	void loadFromJson(Json::Value json)
	{
		x = json.get("x", 0.0).asDouble();
		y = json.get("y", 0.0).asDouble();
		theta = json.get("theta", 0.0).asDouble();
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		json["x"] = x;
		json["y"] = y;
		json["theta"] = theta;
		return json;
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

	void loadFromJson(Json::Value json)
	{
		obs_outline.loadFromJson(json["obs_outline"]);
		pose_table.resize(0);
		for(auto pose : json["pose_table"])
		{
			Pose temp_pose;
			temp_pose.loadFromJson(pose);
			pose_table.push_back(temp_pose);
		}
		dt = json.get("dt", 0.01).asDouble();
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		json["obs_outline"] = obs_outline.saveToJson();
		Json::Value pose_list;
		for(auto pose : pose_table)
		{
			pose_list.append(pose.saveToJson());
		}
		json["pose_table"] = pose_list;
		json["dt"] = dt;
		return json;
	}
};

///
/// A static obstacle.
///
struct ObstacleStatic
{
	Boundary obs_outline;
	Pose obs_pose;

	void loadFromJson(Json::Value json)
	{
		obs_outline.loadFromJson(json["obs_outline"]);
		obs_pose.loadFromJson(json["obs_pose"]);
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		json["obs_outline"] = obs_outline.saveToJson();
		json["obs_pose"] = obs_pose.saveToJson();
		return json;
	}
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

	void loadFromJson(Json::Value json)
	{
		av_outline.loadFromJson(json["av_outline"]);
		av_state_table.resize(0);
		for(auto av_state : json["av_state_table"])
		{
			AvState temp_state;
			temp_state.loadFromJson(av_state);
			av_state_table.push_back(temp_state);
		}
		dt = json.get("dt", 0.01).asDouble();
		av_parameters.loadFromJson(json["av_parameters"]);
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		json["av_outline"] = av_outline.saveToJson();
		Json::Value state_list;
		for(auto state : av_state_table)
		{
			state_list.append(state.saveToJson());
		}
		json["av_state_table"] = state_list;
		json["dt"] = dt;
		json["av_parameters"] = av_parameters.saveToJson();
		return json;
	}
};

///
/// The class for handling trajectory optimization.
///
class Planner
{
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

	void loadFromJson(std::string raw_json);

	std::string saveToJson();

	AvTrajectory solveTrajectory();

private:
	AvState dynamics(AvState input, double turn_rate, double accel_f);

	AvState apply_dynamics(AvState input, AvState current_dynamics, double);

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
