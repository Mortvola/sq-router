/*
 * Search.h
 *
 *  Created on: Oct 29, 2019
 *      Author: richard
 */

#pragma once

#include "Anchor.h"
#include "Graph.h"
#include "SearchEdge.h"
#include "SearchNode.h"
#include "Profiler.h"
#include "ThreadPool.h"
#include <tuple>
#include <vector>
#include <set>

class SearchController;

class Search {
public:
  Search(
    const std::shared_ptr<Graph> &graph,
    SearchController &controller,
    int startIndex,
    int endIndex,
    bool reverseCosts,
    bool log);

  void setOtherSearch(const std::shared_ptr<Search> &otherSearch)
  {
    m_otherSearch = otherSearch;
  }

  void initializeStartNode();

  size_t queueSize()
  {
    std::unique_lock<std::mutex> lock(m_openSearchNodesMutex);
    return m_openSearchNodes.size();
  }

  std::shared_ptr<SearchNode> queueFront()
  {
    std::unique_lock<std::mutex> lock(m_openSearchNodesMutex);

    if (m_openSearchNodes.size() > 0) {
      return *m_openSearchNodes.begin();
    }

    return nullptr;
  }

  void processNext(ThreadPool &threadPool);

  virtual double getNodeSortCost(const std::shared_ptr<SearchNode> &node);

  virtual double getPotentialPathCost(const std::shared_ptr<SearchNode> &node);

  int getStartNodeId()
  {
    return m_startNodeId;
  }

  int getEndNodeId()
  {
    return m_endNodeId;
  }

protected:

  int m_search{-1};

private:

  void overrideEdge(SearchNode &searchNode, int edgeIndex);

  void insertSearchNode(
    const std::shared_ptr<SearchNode> &fromNode,
    const std::shared_ptr<SearchNode> &node);

  std::shared_ptr<SearchNode> traverseEdge(
    const std::shared_ptr<SearchEdge> &searchEdge,
    const std::shared_ptr<SearchNode> &fromNode);

  std::shared_ptr<SearchNode> handleEdgeTraversal(
    const std::shared_ptr<SearchEdge> &searchEdge,
    const std::shared_ptr<SearchNode> &fromNode,
    const std::shared_ptr<SearchNode> &nextNode,
    bool preferredRoute
  );

  std::shared_ptr<SearchNode> handleBetterPath(
    const std::shared_ptr<SearchEdge> &searchEdge,
    const std::shared_ptr<SearchNode> &fromNode,
    const std::shared_ptr<SearchNode> &nextNode,
    double costDelta
  );

  void handleFoundEnd(
    const std::shared_ptr<SearchEdge> &searchEdge,
    const std::shared_ptr<SearchNode> &fromNode,
    const std::shared_ptr<SearchNode> &nextNode
  );

  void setToEndValues(const std::shared_ptr<SearchNode> &node);

  void propogateCostDelta(
    const std::shared_ptr<SearchNode> &node,
    double costDelta);

  bool isSharedNode(const std::shared_ptr<SearchNode> &node);
  void checkSharedNode(const std::shared_ptr<SearchNode> &node);

  double getNodeCost(int nodeId);

  std::shared_ptr<Graph> m_graph;
  SearchController &m_controller;

  struct SearchNodeCompare
  {
    bool operator()(
      const std::shared_ptr<SearchNode> &a,
      const std::shared_ptr<SearchNode> &b) const
    {
      if (a->m_sortCost == b->m_sortCost)
      {
        return a->getNodeId() < b->getNodeId();
      }

      return a->m_sortCost < b->m_sortCost;
    }
  };

  std::mutex m_openSearchNodesMutex;
  std::set<std::shared_ptr<SearchNode>, SearchNodeCompare> m_openSearchNodes;

  bool m_log{false};

  int m_startNodeId{-1};
  int m_endNodeId{-1};

  bool m_reverseCosts{false};

  std::weak_ptr<Search> m_otherSearch;

  Profiler m_profiler3{"3"};
  Profiler m_profiler4{"4"};
  Profiler m_profiler5{"5"};
  Profiler m_profiler6{"6"};
  Profiler m_profiler8{"8"};
  Profiler m_profiler{"processEdge"};
  Profiler m_profiler2{"traverseEdge"};
  Profiler m_profileWait{"Wait"};
  Profiler m_profileProcessNext{"processNext"};
};
