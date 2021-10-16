#pragma once

#include "SearchLogEntry.h"

class SpawnedSearcherLogEntry: public SearchLogEntry
{
public:

  SpawnedSearcherLogEntry(
    int searcherId,
    SearcherState searcherState,
    int search,
    int fromSearcherId,
    const std::shared_ptr<SearchNode> &node,
    bool sharedNode,
    const SearchController &controller,
    double totalCost)
  :
    SearchLogEntry(
      "Spawned",
      searcherId,
      searcherState,
      search,
      fromSearcherId,
      node,
      sharedNode,
      controller,
      totalCost)
  {
  }

};

