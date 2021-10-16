#pragma once

#include "LatLng.h"
#include "DBConnection.h"
#include <tuple>
#include <map>
#include <memory>

namespace gb
{

class Edge
{
public:
  Edge (int id, int lid, int index, double f)
  :
    edgeId(id),
    lineId(lid),
    pointIndex(index),
    fraction(f)
  {}

  Edge (const Edge &other) = default;
  Edge (Edge &&other) = default;
  Edge &operator=(const Edge &other) = default;
  Edge &operator=(Edge &&other) = default;

  int edgeId;
  int lineId;
  int pointIndex;
  double fraction;
  int forwardEdgeId {-1};
  int reverseEdgeId {-1};
private:
};

bool operator==(const Edge &a, const Edge &b);
bool operator<(const Edge &a, const Edge &b);

class Node
{
public:
  Node(
    int id,
    const std::string &w,
    bool sameQuadrangle
  )
  :
    m_nodeId(id),
    m_way(w),
    m_sameQuadrangle(sameQuadrangle)
  {};

  std::vector<Edge> m_currentEdges;

  int m_nodeId {-1};
  std::string m_way;
  bool m_sameQuadrangle {false};
};

class GraphBuilder
{
public:

  GraphBuilder ();

  void buildGraph (
    double lat,
    double lng);

  int updateIntersectionCount(
    DBTransaction &transaction,
    int lat,
    int lng);

private:

  std::shared_ptr<DBConnection> m_dbConnection;

  void getIntersections (
    DBTransaction &transaction,
    double lat,
    double lng,
    pqxx::result &intersections,
    int nodeCount,
    int max);

  void modifyNodes(
    DBTransaction &transaction,
    const std::vector<Node> &existingNodes,
    std::vector<Node> &newNodes,
    int64_t lineId,
    Json::Value &status);

  std::tuple<int, int> getNodeCounts(
    DBTransaction &transaction,
    int lat,
    int lng);

  PreparedStatement m_queryIntersections;
  PreparedStatement m_queryIntersectionPoints;
  PreparedStatement m_countIntersectionPoints;
  PreparedStatement m_queryExistingEdges;
  PreparedStatement m_insertNode;
  PreparedStatement m_deleteEdges;
  PreparedStatement m_intersectionQuery;
  PreparedStatement m_insertCount;
};

} // namespace gb
