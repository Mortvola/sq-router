#pragma once

#include "Cost.h"
#include "Graph.h"
#include "SearchLog.h"
#include "Anchor.h"
#include "Profiler.h"
#include "ThreadPool.h"
#include <map>
#include <vector>

class SplitEdge;
class Search;
class DBConnection;

class SearchController {
public:

  SearchController(
    const std::shared_ptr<Graph> &graph,
    const std::string &algorithm,
    const Point &point1,
    const Point &point2,
    int preferredTrailId,
    bool returnLog);

  std::vector<std::vector<Anchor>> search(ThreadPool &threadPool);

  double getBestCost() const;

  void setBestCost(double cost, int nodeId);

  std::shared_ptr<SearchNode> getSearchNode(int nodeId);

  std::shared_ptr<SearchNode> getSearchNode(const std::shared_ptr<Node> &node);

  double getNodeCost(int nodeId, int search);

  SearchLog &getSearchLog()
  {
    return m_searchLog;
  }

  int createNode(int index);

  bool isEdgePreferred(int startEdgeId, int endEdgeId);

  std::shared_ptr<DBConnection> m_dbConnection;

  std::shared_ptr<Graph> m_graph;

  SearchLog m_searchLog;

private:
  int insertNode(TrailInfo &terminus);

  void overrideEdges();
  
  std::vector<std::shared_ptr<Node>> &nodes() { return m_nodes; }

  std::vector<std::pair<int, int>> m_preferredEdges;

  int m_bestCostNodeId {-1};

  void loadRouteGroup(int preferredTrailId);

  std::vector<Anchor> getRoute(
    const std::shared_ptr<Search> &forwardSearch,
    const std::shared_ptr<Search> &backwardSearch);

  std::vector<Anchor> generateAnchors(
    int startNodeId,
    int endNodeId,
    int direction);

  std::vector<Anchor> BiDiDijkstra(ThreadPool &threadPool);
  std::vector<Anchor> AStar(ThreadPool &threadPool);
  std::vector<Anchor> BiDiAStar(ThreadPool &threadPool);

  void addNewAnchor(
    std::vector<Anchor> &newAnchors,
    Anchor &anchor,
    int direction);

  std::mutex m_searchNodesAccess;
  std::map<int, std::shared_ptr<SearchNode>> m_searchNodes;

  std::vector<std::shared_ptr<SplitEdge>> m_splits;
  // int m_splitEdgeSeq{-512};
  int m_insertedNodeSeq{-2};

  std::vector<std::shared_ptr<Node>> m_nodes;
  std::vector<std::shared_ptr<Edge>> m_edges;

  double bestCost{-1};

  std::string m_algorithm;
  std::vector<Point> m_points;
  bool m_returnLog {false};

  Profiler m_getNodeProfiler{"getNode"};
  Profiler m_searchProfiler{"overall search"};
  Profiler m_getRouteProfiler{"getRoute"};
};
