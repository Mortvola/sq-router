/*
 * Graph.h
 *
 *  Created on: Oct 26, 2019
 *      Author: richard
 */

#pragma once

#include "Elevation.h"
#include "Node.h"
#include "Edge.h"
#include "Map.h"
#include "DBConnection.h"
#include "Profiler.h"
#include <pqxx/pqxx>
#include <map>
#include <list>
#include <chrono>
#include <iostream>
#include <mutex>
#include <shared_mutex>

class NavEdge
{
public:
  int m_edgeId;
  int m_nodeId;
  int m_otherEdgeId[2];

  std::shared_ptr<Edge> m_edge[2];

private:

};

class Graph
{
public:

	Graph ();
	~Graph () = default;

	Graph (const Graph &other) = delete;
	Graph (Graph &&other) = delete;
	Graph &operator= (const Graph &other) = delete;
	Graph &operator= (Graph &&other) = delete;

	static constexpr int NodeIdNone {-1};
	static constexpr int EdgeIdNone {-1};

	std::shared_ptr<Node> getNode (int nodeId)
	{
    std::shared_ptr<Node> node;

    {
      std::unique_lock<std::mutex> lock(m_graphAccess);

      node = getNodeNoLock(nodeId);
    }

    if (!node)
    {
      loadNode(nodeId);

      std::unique_lock<std::mutex> lock(m_graphAccess);

      node = getNodeNoLock(nodeId);
    }
  
    return node;
	}

  double distanceBetweenNodes(const Node &node1, const Node &node2);
	double costBetweenNodes(const Node &node1, const Node &node2);

	void loadNode(int nodeId);

//	int removeDeadEndNodesAndEdges (const std::shared_ptr<Node> &node);

	std::chrono::duration<double> m_nodeLoadingTime {};
	std::chrono::duration<double> m_edgeQueryTime {};

  Profiler m_loadNodes;

private:

  void loadNodeNoLock(int nodeId);

	std::shared_ptr<Node>  getNodeNoLock (int nodeId)
	{
    try
    {
  		return m_nodes.at(nodeId);
    }
    catch(const std::exception& e)
    {
    }

    return nullptr;
	}

	std::tuple<double, int, int> traverseEdge(
			int edgeId, int prevNodeId);
	double getCostToNode(int prevNodeId, const Edge &edge);

//	int removeDeadEndNodesAndEdgesNoLock (const std::shared_ptr<Node> &node);

	std::mutex m_graphAccess;
	std::map<int, std::shared_ptr<Node>> m_nodes;
	std::map<int, NavEdge> m_edges;

  std::shared_ptr<DBConnection> m_dbConnection;

  PreparedStatement m_queryNode;
  PreparedStatement m_queryArea;
};
