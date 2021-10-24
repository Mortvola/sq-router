#pragma once

#include "LatLngBounds.h"
#include "DBConnection.h"
#include <tuple>
#include <map>
#include <memory>

namespace gb
{

class Edge
{
public:
  Edge (int lid, int index, double f)
  :
    lineId(lid),
    pointIndex(index),
    fraction(f)
  {}

  Edge (const Edge &other) = default;
  Edge (Edge &&other) = default;
  Edge &operator=(const Edge &other) = default;
  Edge &operator=(Edge &&other) = default;

  int id {-1};
  int lineId {-1};
  int pointIndex {-1};
  int nodeId {-1};
  double fraction {0};
  double forwardCost {0};
  double reverseCost {0};
  int forwardEdgeId {-1};
  int reverseEdgeId {-1};

private:
};

bool operator==(const Edge &a, const Edge &b);
bool operator<(const Edge &a, const Edge &b);

struct EdgeUpdate
{
  enum class Operation
  {
    update,
    insert
  };

  EdgeUpdate(Operation o, Edge *e)
  :
    operation(o),
    edge(e)
  {}

  Operation operation;
  Edge *edge;
};

class Node
{
public:
  Node()
  {
  };

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

  std::vector<Edge> m_edges;

  int m_nodeId {-1};
  std::string m_way;
  bool m_sameQuadrangle {false};
};

class ExistingNode
{
public:
  ExistingNode(
    int id,
    const std::string &w,
    bool sameQuadrangle
  )
  :
    m_nodeId(id),
    m_way(w),
    m_sameQuadrangle(sameQuadrangle)
  {};

  std::vector<Edge> m_edges;

  int m_nodeId;
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

  void buildGraphInArea(
    const LatLngBounds &bounds);

  int updateIntersectionCount(
    DBTransaction &transaction,
    int lat,
    int lng);

private:

  std::shared_ptr<DBConnection> m_dbConnection;

  void buildGraphInArea(
    DBTransaction &transaction,
    const LatLngBounds &bounds,
    int nodeCount,
    int max);

  void updateIntersections (
    DBTransaction &transaction,
    const LatLngBounds &bounds,
    pqxx::result &intersections,
    int nodeCount,
    int max);

  void modifyNodes(
    DBTransaction &transaction,
    const std::vector<ExistingNode> &existingNodes,
    std::vector<Node> &newNodes,
    int64_t lineId,
    Json::Value &status);

  std::tuple<int, int> getNodeCounts(
    DBTransaction &transaction,
    int lat,
    int lng);

  std::vector<Node> getProposedNodes(
    DBTransaction &transaction,
    int64_t lineId,
    const std::string &others,
    const LatLngBounds &bounds,
    Json::Value &status);

  std::vector<ExistingNode> getExistingNodes (
    DBTransaction &transaction,
    int64_t lineId,
    const LatLngBounds &bounds);

  std::tuple<Node, std::vector<Json::Value>> makeNodeFromRow (const pqxx::row &row, const LatLngBounds &bounds);

  struct EditLists
  {
    std::vector<EdgeUpdate> lineEdges;
    std::vector<EdgeUpdate> otherEdges;
    std::string deleteEdges;
    std::string deleteNodes;
  };

  EditLists makeEditLists(
    DBTransaction &transaction,
    const std::vector<ExistingNode> &existingNodes,
    std::vector<Node> &proposedNodes,
    int64_t lineId,
    Json::Value &status);

  void setEdgeRelationsAndCosts(
    DBTransaction &transaction,
    int lineId,
    std::vector<EdgeUpdate> &lineEdges);

  std::tuple<double, double> getEdgeCosts(
    DBTransaction &transaction,
    int lineId,
    double startFraction,
    double endFraction);

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
