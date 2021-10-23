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

  std::vector<Edge> m_currentEdges;

  int m_nodeId {-1};
  std::string m_way;
  bool m_sameQuadrangle {false};
};

enum class Operation
{
  update,
  insert
};

struct UpdateRec
{
  UpdateRec(int id, int pi, Operation o, int ni, const Edge *e1, const Edge *e2)
  :
    edgeId (id),
    pointIndex(pi),
    operation(o),
    nodeId(ni),
    edge1(e1),
    edge2(e2)
  {}

  int edgeId {-1};
  int pointIndex;
  Operation operation;
  int nodeId;
  const Edge *edge1;
  const Edge *edge2;
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
    const std::vector<Node> &existingNodes,
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

  std::vector<Node> getExistingNodes (
    DBTransaction &transaction,
    int64_t lineId,
    const LatLngBounds &bounds);

  std::tuple<Node, std::vector<Json::Value>> makeNodeFromRow (const pqxx::row &row, const LatLngBounds &bounds);

  struct EditLists
  {
    std::vector<UpdateRec> lineEdges;
    std::stringstream updateEdges;
    std::stringstream insertEdges;
    std::string deleteEdges;
    std::string deleteNodes;
  };

  EditLists makeEditLists(
    DBTransaction &transaction,
    const std::vector<Node> &existingNodes,
    std::vector<Node> &proposedNodes,
    int64_t lineId,
    Json::Value &status);

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
