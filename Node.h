/*
 * Node.h
 *
 *  Created on: Oct 26, 2019
 *      Author: richard
 */

#pragma once

#include <vector>
#include "Point.h"
#include "Edge.h"
#include "ThreadPool.h"
#include <map>
#include <shared_mutex>
#include <atomic>
#include <algorithm>
#include <future>

class Node
{
public:

	Node (int nodeId)
	:
		m_nodeId (nodeId)
	{
	}

	~Node () = default;

	Node (const Node &other) = delete;
	Node (Node &&other) = delete;
	Node &operator= (const Node &other) = default;
	Node &operator= (Node &&other) = default;

	int getNodeId ()
	{
		return m_nodeId;
	}

	void addEdge (const std::shared_ptr<Edge> &newEdge);

	size_t edgeCount ()
	{
  	std::unique_lock<std::mutex> lock(m_edgesAccess);

		return m_edges.size();
	}

  template<class T>
  std::vector<std::future<T>> forEachEdge(
    ThreadPool &threadPool,
    std::function<T(const std::shared_ptr<Edge> &)> callback);

	std::vector<std::shared_ptr<Edge>> &edges ()
	{
		std::unique_lock<std::mutex> lock(m_edgesAccess);

		return m_edges;
	}

  std::shared_ptr<Edge> getEdge(int otherNodeId, int lineId)
  {
		std::unique_lock<std::mutex> lock(m_edgesAccess);

    auto iter = std::find_if(m_edges.begin(), m_edges.end(),
      [otherNodeId, lineId](const std::shared_ptr<Edge> &edge)
      {
        return edge->m_lineId == lineId
          && (edge->m_edgeEnd[Edge::EndType::Start].m_nodeId == otherNodeId
          || edge->m_edgeEnd[Edge::EndType::End].m_nodeId == otherNodeId);
      }
    );

    if (iter != m_edges.end())
    {
      return *iter;
    }

    return nullptr;
  }

	std::unique_lock<std::mutex> getUniqueLock ()
	{
		return std::unique_lock<std::mutex>(m_nodeAccess);
	}

	friend std::ostream& operator<<(std::ostream &os, const Node &node)
	{
		os << std::setprecision(15) <<
			"node ID: " << node.m_nodeId;

		std::unique_lock<std::mutex> lock(node.m_edgesAccess);

		os << ", edges: (";

		for (auto &edgeId: node.m_edges)
		{
			os << edgeId << ",";
		}

		os << ")";

		os << std::endl;

		return os;
	}

	Point m_point;

private:

	int m_nodeId;

	mutable std::mutex m_edgesAccess;
	std::vector<std::shared_ptr<Edge>> m_edges;

	std::mutex m_nodeAccess;
};

template<class T>
std::vector<std::future<T>> Node::forEachEdge(
  ThreadPool &threadPool,
  std::function<T(const std::shared_ptr<Edge> &)> callback)
{
  std::vector<std::future<T>> futures;

  std::lock_guard<std::mutex> lock(m_edgesAccess);

  for (auto &edge: m_edges)
  {
    auto future = threadPool.postTask<T>(
      [edge, callback]
      {
        return callback(edge);
      }
    );

    futures.push_back(std::move(future));
  }

  return futures;
}


void updateNavNodeElevations();
