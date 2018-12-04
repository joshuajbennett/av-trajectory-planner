#ifndef AVSTRUCTS_HPP
#define AVSTRUCTS_HPP

#include <jsoncpp/json/json.h>
#include <math.h>
#include <vector>

namespace av_structs
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

	enum Indices
	{
		X,
		Y,
		PSI,
		DELTA_F,
		VEL_F,
		SIZE
	};

	AvState operator*(double val)
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
/// Contains all actions variables needed to define the action applied to an AV.
///
struct AvAction
{
	double turn_rate;
	double accel_f;

	enum Indices
	{
		TURN_RATE,
		ACCEL_F,
		SIZE
	};
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

	enum Indices
	{
		X,
		Y,
		SIZE
	};

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
		json["vertices"] = point_list;
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
	double psi;

	enum Indices
	{
		X,
		Y,
		PSI,
		SIZE
	};

	Pose operator*(double val)
	{
		return std::move(Pose {x * val, y * val, psi * val});
	}

	Pose operator+(Pose pose)
	{
		return std::move(Pose {x + pose.x, y + pose.y, psi + pose.psi});
	}

	void loadFromJson(Json::Value json)
	{
		x = json.get("x", 0.0).asDouble();
		y = json.get("y", 0.0).asDouble();
		psi = json.get("psi", 0.0).asDouble();
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		json["x"] = x;
		json["y"] = y;
		json["psi"] = psi;
		return json;
	}
};

///
/// An obstacle trajectory.
///
struct ObstacleTrajectory
{
	Boundary outline;
	std::vector<Pose> table;
	double dt;

	Pose interpolate(double time)
	{
		if(time < 0)
		{
			return std::move(table[0]);
		}
		else if(time >= (table.size() - 1) * dt)
		{
			return std::move(table[table.size() - 1]);
		}
		else
		{
			int first_pos = floor(time / dt);
			Pose first = table[first_pos];
			Pose second = table[first_pos + 1];
			double dist = time / dt - first_pos;
			double inv_dist = 1.0 - dist;
			return std::move(first * inv_dist + second * dist);
		}
	}

	void loadFromJson(Json::Value json)
	{
		outline.loadFromJson(json["outline"]);
		table.resize(0);
		for(auto pose : json["table"])
		{
			Pose temp_pose;
			temp_pose.loadFromJson(pose);
			table.push_back(temp_pose);
		}
		dt = json.get("dt", 0.01).asDouble();
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		json["outline"] = outline.saveToJson();
		Json::Value pose_list;
		for(auto pose : table)
		{
			pose_list.append(pose.saveToJson());
		}
		json["table"] = pose_list;
		json["dt"] = dt;
		return json;
	}
};

///
/// A static obstacle.
///
struct ObstacleStatic
{
	Boundary outline;
	Pose obs_pose;

	void loadFromJson(Json::Value json)
	{
		outline.loadFromJson(json["outline"]);
		obs_pose.loadFromJson(json["obs_pose"]);
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		json["outline"] = outline.saveToJson();
		json["obs_pose"] = obs_pose.saveToJson();
		return json;
	}
};

///
/// An AV trajectory.
///
struct AvTrajectory
{
	Boundary outline;
	std::vector<AvState> table;
	double dt;
	AvParams av_parameters;

	AvState interpolate(double time)
	{
		if(time < 0)
		{
			return std::move(table[0]);
		}
		else if(time >= (table.size() - 1) * dt)
		{
			return std::move(table[table.size() - 1]);
		}
		else
		{
			int first_pos = floor(time / dt);
			AvState first = table[first_pos];
			AvState second = table[first_pos + 1];
			double dist = time / dt - first_pos;
			double inv_dist = 1.0 - dist;
			return std::move(first * inv_dist + second * dist);
		}
	}

	void loadFromJson(Json::Value json)
	{
		outline.loadFromJson(json["outline"]);
		table.resize(0);
		for(auto av_state : json["table"])
		{
			AvState temp_state;
			temp_state.loadFromJson(av_state);
			table.push_back(temp_state);
		}
		dt = json.get("dt", 0.01).asDouble();
		av_parameters.loadFromJson(json["av_parameters"]);
	}

	Json::Value saveToJson()
	{
		Json::Value json;
		json["outline"] = outline.saveToJson();
		Json::Value state_list;
		for(auto state : table)
		{
			state_list.append(state.saveToJson());
		}
		json["table"] = state_list;
		json["dt"] = dt;
		json["av_parameters"] = av_parameters.saveToJson();
		return json;
	}
};

} // namespace av_structs

#endif
