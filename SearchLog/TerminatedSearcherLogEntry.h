#pragma once

#include "SearchLogEntry.h"

class TerminatedSearcherLogEntry: public SearchLogEntry
{
public:

  TerminatedSearcherLogEntry(
    int search,
    int fromSearcherId,
    const std::shared_ptr<SearchNode> &node,
    const SearchController &controller,
    double totalCost)
  :
    SearchLogEntry(
      "Terminated",
      -1,
      SearchLogEntry::SearcherState::Inactive,
      search,
      fromSearcherId,
      node,
      false,
      controller,
      totalCost)
  {
  }
};

