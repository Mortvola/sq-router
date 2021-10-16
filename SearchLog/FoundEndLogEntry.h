#pragma once

#include "SearchLogEntry.h"

class FoundEndLogEntry: public SearchLogEntry
{
public:

  FoundEndLogEntry(
    int search,
    int fromSearcherId,
    const std::shared_ptr<SearchNode> &node,
    const SearchController &controller,
    double totalCost)
  :
    SearchLogEntry(
      "Found End",
      -1,
      SearchLogEntry::SearcherState::Inactive,
      search,
      fromSearcherId,
      node,
      true,
      controller,
      totalCost)
  {
  }

};
