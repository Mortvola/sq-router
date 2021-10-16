#pragma once

#include "SearchLogEntry.h"

class BlockedLogEntry: public SearchLogEntry
{
public:

  BlockedLogEntry(
    int search,
    int fromSearcherId,
    const std::shared_ptr<SearchNode> &node,
    const SearchController &controller)
  :
    SearchLogEntry(
      "Blocked",
      -1,
      SearchLogEntry::SearcherState::Inactive,
      search,
      fromSearcherId,
      node,
      false,
      controller,
      0)
  {
  }

};
