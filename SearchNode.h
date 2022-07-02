/*
 * SearchNode.h
 *
 *  Created on: Oct 29, 2019
 *      Author: richard
 */

#pragma once

#include "Edge.h"
#include "Node.h"
#include "SearchEdge.h"
#include "ThreadPool.h"
#include <set>
#include <vector>
#include <memory>
#include <future>

class SearchController;
class Graph;

class SearchNode: public std::enable_shared_from_this<SearchNode>
{
public:
  explicit SearchNode(
    SearchController &controller,
    const std::shared_ptr<Node> &node)
  :
    m_controller(controller),
    m_node(node)
  {}

  ~SearchNode() = default;

  SearchNode(const SearchNode &other) = delete;
  SearchNode(SearchNode &&other) = delete;
  SearchNode &operator=(const SearchNode &other) = delete;
  SearchNode &operator=(SearchNode &&other) = delete;

  int getNodeId() const
  {
    if (m_node)
    {
      return m_node->getNodeId();
    }

    return -1;
  }

  std::shared_ptr<Node> getNode() const
  {
    return m_node;
  }

  int edgeCount() const {
    if (m_node)
    {
      return m_node->edgeCount();
    }

    return 0;
  }

  bool hasNonTraversedEdges();

  void forEachEdge(
    std::function<void(const std::shared_ptr<Edge> &)> callback);

  std::vector<std::future<std::shared_ptr<SearchNode>>> forEachEdge(
    ThreadPool &threadPool,
    std::function<std::shared_ptr<SearchNode>(const std::shared_ptr<Edge> &)> callback);

  std::vector<std::shared_ptr<SearchEdge>> getSearchEdges();

  Point getPoint() const { return m_node->m_point; }

  std::shared_ptr<SearchEdge> getEntryEdge(int search);

  std::unique_lock<std::mutex> getUniqueLock()
  {
    return std::unique_lock<std::mutex>(m_nodeAccess);
  }

  template<class T>
  std::unique_lock<std::mutex> getUniqueSearchEdgeLock(T t)
  {
    return std::unique_lock<std::mutex>(m_searchEdgesAccess, t);
  }

  template<class T>
  std::unique_lock<std::mutex> getUniqueLock(T t)
  {
    return std::unique_lock<std::mutex>(m_nodeAccess, t);
  }

  friend std::ostream &operator<<(std::ostream &os, const SearchNode &node)
  {
    os << *node.m_node;

    return os;
  }

  std::shared_ptr<SearchEdge> createSearchEdge(
    const std::shared_ptr<Edge> &edge);

  std::shared_ptr<SearchEdge> getSearchEdge(const std::shared_ptr<Edge> &edge);

  std::shared_ptr<Edge> getEdge(int otherNodeId, int lineId) const
  {
    if (m_node == nullptr)
    {
      throw std::runtime_error("node pointer is null");
    }

    return m_node->getEdge(otherNodeId, lineId);
  }

  LatLng getLatLng()
  {
    return { m_node->m_point.m_lat, m_node->m_point.m_lng };
  }

  double getCumulativeCost(int searchDirection)
  {
    return m_searchInfo[searchDirection].m_cummulativeCost;
  }

  void setCumulativeCost(int searchDirection, double cumulativeCost)
  {
    m_searchInfo[searchDirection].m_cummulativeCost = cumulativeCost;
  }

  double getTimeToEnd(int searchDirection)
  {
    return m_searchInfo[searchDirection].m_timeToEnd;
  }

  void setTimeToEnd(int searchDirection, const std::shared_ptr<Graph> &graph, int endNodeId);

  SearchController &m_controller;

  struct SearchInfo
  {
    double m_cummulativeCost{-1};
    // double m_costToEnd{-1};
    double m_timeToEnd{-1};
    int m_queued{0};
    int m_visitCount{0};
  };

  SearchInfo m_searchInfo[2];

  std::shared_ptr<Node> m_node;

  std::vector<std::pair<std::shared_ptr<Edge>, std::shared_ptr<Edge>>> m_edgeOverrides;

  std::shared_ptr<SearchNode> m_fromNode;

  double m_sortCost{0};

  bool m_preferredNode{false};

private:

  std::shared_ptr<Edge> getEdgeOverride(const std::shared_ptr<Edge> &edge);

  std::shared_ptr<SearchEdge> getSearchEdgeNoLock(const std::shared_ptr<Edge> &edge);

  std::mutex m_nodeAccess;

  mutable std::mutex m_searchEdgesAccess;
  std::vector<std::shared_ptr<SearchEdge>> m_searchEdges;
};
