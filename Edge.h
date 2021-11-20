/*
 * Edge.h
 *
 *  Created on: Oct 26, 2019
 *      Author: richard
 */

#pragma once

#include "EdgeEnd.h"
#include <atomic>
#include <iostream>
#include <iomanip>
#include <memory>

class Node;

class Edge
{
public:

	Edge () = default;
	~Edge () = default;

	Edge (const Edge &other) = delete;
	Edge (Edge &&other) = delete;
	Edge &operator= (const Edge &other) = delete;
	Edge &operator= (Edge &&other) = delete;

  enum EndType
  {
    Start = 0,
    End = 1,
  };

	int getOtherNodeId(int nodeId) const;
  
  std::shared_ptr<Node> getOtherNode(const std::shared_ptr<Node> &node);

	double getCost (int startNodeId, bool reverseCost) const;

	friend std::ostream& operator<<(std::ostream &os, const Edge &edge)
	{
		os << std::setprecision(15) <<
			", start node: " << edge.m_edgeEnd[EndType::Start].m_nodeId <<
			", end node: " << edge.m_edgeEnd[EndType::End].m_nodeId;

		return os;
	}

	int m_pointIndex {-1};
	int m_lineId {-1};
  EdgeEnd m_edgeEnd[2];
};

