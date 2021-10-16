#include "SearchLogEntry.h"
#include "SearchController.h"
#include "SearchNode.h"

static int entryID {0};

SearchLogEntry::SearchLogEntry(
  const std::string &type,
  int searcherId,
  SearcherState searcherState,
  int search,
  int fromSearcherId,
  const std::shared_ptr<SearchNode> &nextNode,
  bool sharedNode,
  const SearchController &controller,
  double totalCost)
:
  m_entryID(entryID++),
  m_type(type),
  m_searcherId(searcherId),
  m_searcherState(searcherState),
  m_search(search),
  m_fromSearcherId(fromSearcherId),
  m_nodeId(nextNode->getNodeId()),
  m_sharedNode(sharedNode),
  m_point(nextNode->m_node->m_point),
  m_cost(nextNode->m_searchInfo[m_search].m_cummulativeCost),
  m_costToEnd(nextNode->m_searchInfo[m_search].m_timeToEnd),
  m_bestCost(controller.getBestCost()),
  m_edgeCount(nextNode->edgeCount()),
  m_totalCost(totalCost)
{
}

SearchLogEntry::SearchLogEntry(
  const std::string &type,
  int searcherId,
  SearcherState searcherState,
  int search,
  int fromSearcherId,
  const SearchController &controller,
  double totalCost)
:
  m_entryID(entryID++),
  m_type(type),
  m_searcherId(searcherId),
  m_searcherState(searcherState),
  m_search(search),
  m_fromSearcherId(fromSearcherId),
  m_nodeId(-1),
  m_sharedNode(false),
  m_point(),
  m_cost(0),
  m_costToEnd(0),
  m_bestCost(controller.getBestCost()),
  m_edgeCount(0),
  m_totalCost(totalCost)
{
}

SearchLogEntry::operator Json::Value() const
{
  Json::Value value;

  value["entryId"] = m_entryID;
  value["type"] = m_type;
  value["id"] = m_searcherId;
  value["searchState"] = searchStateString();
  value["search"] = m_search;
  value["from"] = m_fromSearcherId;
  value["nodeId"] = m_nodeId;
  value["sharedNode"] = m_sharedNode;
  value["point"] = m_point;
  value["cost"] = m_cost;
  value["costToEnd"] = m_costToEnd;
  value["bestCost"] = m_bestCost;
  value["edgeCount"] = m_edgeCount;
  value["totalCost"] = m_totalCost;

  return value;
}

