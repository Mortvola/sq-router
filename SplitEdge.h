#pragma once

#include "Edge.h"
#include <vector>
#include <memory>

class Node;

class SplitEdge: public Edge
{
public:

	SplitEdge () = default;
	~SplitEdge () = default;

	SplitEdge (const SplitEdge &other) = delete;
	SplitEdge (SplitEdge &&other) = delete;
	SplitEdge &operator= (const SplitEdge &other) = delete;
	SplitEdge &operator= (SplitEdge &&other) = delete;

  int m_lineId {-1};
	double m_startFraction {-1};
	double m_endFraction {-1};

  std::vector<std::pair<double, std::shared_ptr<Node>>> m_nodes;

	int m_startEdgeId {-1};
	int m_endEdgeId {-1};
	int m_midNode {-1};
};


