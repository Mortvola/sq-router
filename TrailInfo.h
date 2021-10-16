#pragma once

#include <napi.h>

class TrailInfo
{
public:

	TrailInfo () = default;
	~TrailInfo () = default;

	TrailInfo (const TrailInfo &other) = default;
	TrailInfo (TrailInfo &&other) = default;
	TrailInfo &operator= (const TrailInfo &other) = delete;
	TrailInfo &operator= (TrailInfo &&other) = delete;

	operator Json::Value () const
	{
		Json::Value value;

		value["distance"] = m_distance;
		value["point"] = m_point;
		value["fraction"] = m_fraction;
		value["start_edge_id"] = m_startEdgeId;
		value["end_edge_id"] = m_endEdgeId;
		value["start_fraction"] = m_startFraction;
		value["end_fraction"] = m_endFraction;
		value["start_node"] = m_startNodeId;
		value["end_node"] = m_endNodeId;
		value["line_id"] = m_lineId;
		value["forwardCost"] = m_forwardCost;
		value["backwardCost"] = m_backwardCost;
		value["highway"] = m_highway;
		value["name"] = m_name;
		value["surface"] = m_surface;

		return value;
	}

	double m_distance {-1};
	Point m_point;
	double m_fraction {-1};
	int m_startEdgeId {-1};
	int m_endEdgeId {-1};
	double m_startFraction {-1};
	double m_endFraction {-1};
	int m_startNodeId {-1};
	int m_endNodeId {-1};
	int m_lineId {-1};
	double m_forwardCost {-1};
	double m_backwardCost {-1};
	std::string m_highway;
	std::string m_name;
	std::string m_surface;
};
