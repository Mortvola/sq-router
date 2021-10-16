#pragma once

#include "SearchLogEntry.h"

class DeadEndLogEntry: public SearchLogEntry
{
public:

  DeadEndLogEntry(
    int search,
    int fromSearcherId,
    const SearchController &controller)
  :
    SearchLogEntry(
      "Dead End",
      -1,
      SearchLogEntry::SearcherState::Inactive,
      search,
      fromSearcherId,
      controller,
      0)
  {
  }

};
