#pragma once

#include "LatLngBounds.h"
#include "DBConnection.h"
#include "Node.h"
#include "Edge.h"
#include "EdgeUpdate.h"
#include <tuple>
#include <map>
#include <memory>
#include <thread>
#include <condition_variable>
#include <deque>

namespace gb
{

class GraphBuilder
{
public:

  GraphBuilder ();

  void start();

  void postRequest(const LatLngBounds &bounds);

  void deleteRequest(const LatLngBounds &bounds);

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

	std::mutex m_generateQueueMutex;
  std::deque<LatLngBounds> m_generateQueue;

	std::thread m_generateThread;
	std::condition_variable m_generateCondition;

  void generate();

  std::shared_ptr<DBConnection> m_dbConnection;

  bool updateIntersections (
    DBTransaction &transaction,
    const LatLngBounds &bounds,
    pqxx::result &intersections);

  void modifyNodes(
    DBTransaction &transaction,
    const std::vector<Node> &existingNodes,
    std::vector<Node> &newNodes,
    int64_t lineId);

  std::tuple<int, int> getNodeCounts(
    DBTransaction &transaction,
    int lat,
    int lng);

  std::vector<Node> getProposedNodes(
    DBTransaction &transaction,
    int64_t lineId,
    const LatLngBounds &bounds);

  std::vector<Node> getExistingNodes (
    DBTransaction &transaction,
    int64_t lineId,
    const LatLngBounds &bounds);

  std::tuple<Node, std::vector<Json::Value>> makeNodeFromRow (
    DBTransaction &transaction,
    const pqxx::row &row,
    const LatLngBounds &bounds);

  struct EditLists
  {
    std::vector<EdgeUpdate> lineEdges;
    std::vector<EdgeUpdate> otherEdges;
    std::string deleteEdges;
    std::string deleteNodes;
  };

  EditLists makeEditLists(
    DBTransaction &transaction,
    const std::vector<Node> &existingNodes,
    std::vector<Node> &proposedNodes,
    int64_t lineId);

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