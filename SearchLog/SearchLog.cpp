#include "SearchLog.h"
#include "SearchNode.h"
#include "SearchController.h"
#include "SpawnedSearcherLogEntry.h"
#include "TerminatedSearcherLogEntry.h"
#include "FoundEndLogEntry.h"
#include "BlockedLogEntry.h"
#include "DeadEndLogEntry.h"

SearchLog::SearchLog(SearchController &controller)
:
  m_controller(controller)
{
}

SearchLog::~SearchLog()
{
  std::cout << "Destroying SearchLog" << std::endl;
}

Json::Value SearchLog::getAsJson()
{
  Json::Value events;
 
  {
    Json::Value array = Json::arrayValue;

    std::unique_lock<std::mutex> lock(m_entriesMutex);

    for (const auto &entry: m_entries)
    {
      array.append(static_cast<Json::Value>(*entry));
    }

    events["events"] = array;
  }

  events["edges"] = Json::arrayValue;

  for (int i = 0; i < 2; i++)
  {
    Json::Value array = Json::arrayValue;

    for (const auto &entry: m_searchEdges[i])
    {
      Json::Value edge = Json::arrayValue;

      for (int j = 0; j < 2; j++)
      {
        Json::Value point = Json::arrayValue;

        point.append(entry[j][0]);
        point.append(entry[j][1]);

        edge.append(point);
      }

      array.append(edge);
    }

    events["edges"][i] = array;
  }

  return events;
}

void SearchLog::addEdge(
  int search,
  const std::shared_ptr<SearchNode> &fromNode,
  const std::shared_ptr<SearchNode> &toNode)
{
    if (fromNode->m_node == nullptr || toNode->m_node == nullptr)
    {
      throw std::runtime_error("node pointer is null");
    }
  
    std::vector<double> point1 {fromNode->m_node->m_point.m_lat, fromNode->m_node->m_point.m_lng};
    std::vector<double> point2 {toNode->m_node->m_point.m_lat, toNode->m_node->m_point.m_lng};
    std::vector<std::vector<double>> edge {point1, point2};

    m_searchEdges[search].push_back(edge);
}

void SearchLog::addSpawnedSearcherEntry(
  int search,
  int searcherId,
  SearchLogEntry::SearcherState searcherState,
  int fromSearcherId,
  const std::shared_ptr<SearchNode> &fromNode,
  const std::shared_ptr<SearchNode> &toNode,
  bool sharedNode,
  double totalCost)
{
  std::unique_lock<std::mutex> lock(m_entriesMutex);

  m_entries.push_back(std::make_unique<SpawnedSearcherLogEntry>(
    searcherId,
    searcherState,
    search,
    fromSearcherId,
    toNode,
    sharedNode,
    m_controller,
    totalCost));

  addEdge(search, fromNode, toNode);
}

void SearchLog::addTerminatedSearcherEntry(
  int search,
  int fromSearcherId,
  const std::shared_ptr<SearchNode> &fromNode,
  const std::shared_ptr<SearchNode> &toNode,
  double totalCost)
{
  std::unique_lock<std::mutex> lock(m_entriesMutex);

  m_entries.push_back(std::make_unique<TerminatedSearcherLogEntry>(
    search,
    fromSearcherId,
    toNode,
    m_controller,
    totalCost));

  addEdge(search, fromNode, toNode);
}

void SearchLog::addFoundEndEntry(
  int search,
  int fromSearcherId,
  const std::shared_ptr<SearchNode> &fromNode,
  const std::shared_ptr<SearchNode> &toNode,
  double totalCost)
{
  std::unique_lock<std::mutex> lock(m_entriesMutex);

  m_entries.push_back(std::make_unique<FoundEndLogEntry>(
    search,
    fromSearcherId,
    toNode,
    m_controller,
    totalCost));

  addEdge(search, fromNode, toNode);
}

void SearchLog::addBlockedEntry(
  int search,
  int fromSearcherId,
  const std::shared_ptr<SearchNode> &fromNode,
  const std::shared_ptr<SearchNode> &toNode)
{
  std::unique_lock<std::mutex> lock(m_entriesMutex);

  m_entries.push_back(std::make_unique<BlockedLogEntry>(
    search,
    fromSearcherId,
    toNode,
    m_controller));
}

void SearchLog::addDeadEndEntry(
  int search,
  int fromSearcherId)
{
  std::unique_lock<std::mutex> lock(m_entriesMutex);

  m_entries.push_back(std::make_unique<DeadEndLogEntry>(
    search,
    fromSearcherId,
    m_controller));
}

