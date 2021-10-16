#pragma once

#include <SearchLogEntry.h>
#include <vector>
#include <mutex>

class SearchController;
class SearchNode;

class SearchLog
{
public:

  SearchLog(SearchController &controller);
  ~SearchLog();

  Json::Value getAsJson();

  void addSpawnedSearcherEntry(
    int search,
    int searcherId,
    SearchLogEntry::SearcherState searcherState,
    int fromSearcherId,
    const std::shared_ptr<SearchNode> &fromNode,
    const std::shared_ptr<SearchNode> &toNode,
    bool sharedNode,
    double totalCost);

  void addTerminatedSearcherEntry(
    int search,
    int fromSearcherId,
    const std::shared_ptr<SearchNode> &fromNode,
    const std::shared_ptr<SearchNode> &toNode,
    double totalCost);

  void addFoundEndEntry(
    int search,
    int fromSearcherId,
    const std::shared_ptr<SearchNode> &fromNode,
    const std::shared_ptr<SearchNode> &toNode,
    double totalCost);

  void addBlockedEntry(
    int search,
    int fromSearcherId,
    const std::shared_ptr<SearchNode> &fromNode,
    const std::shared_ptr<SearchNode> &toNode);

  void addDeadEndEntry(
    int search,
    int fromSearcherId);

private:

  void addEdge(
    int search,
    const std::shared_ptr<SearchNode> &fromNode,
    const std::shared_ptr<SearchNode> &toNode);

  SearchController &m_controller;

  std::mutex m_entriesMutex;
  std::vector<std::unique_ptr<SearchLogEntry>> m_entries;
  std::vector<std::vector<std::vector<double>>> m_searchEdges[2];
};
