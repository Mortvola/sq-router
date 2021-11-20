#pragma once

#include "SearchLogNode.h"
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

  operator Json::Value () const;

  void startNewEntry()
  {
    m_entries.push_back({});
  }

  void addOpenNode(const std::shared_ptr<SearchNode> &node, int searchDirection)
  {
    m_entries[m_entries.size() - 1].addOpenNode(node, searchDirection);
  }

  void removeOpenNode(int nodeId)
  {
    m_entries[m_entries.size() - 1].removeOpenNode(nodeId);
  }

  std::vector<SearchLogEntry> entries()
  {
    return m_entries;
  }

private:

  SearchController &m_controller;

  std::mutex m_entriesMutex;
  std::vector<SearchLogEntry> m_entries;
};
