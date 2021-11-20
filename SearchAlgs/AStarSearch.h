#pragma once

#include "Search.h"

class AStarSearch: public Search
{
public:
  AStarSearch(
    const std::shared_ptr<Graph> &graph,
    SearchController &controller,
    int startIndex,
    int endIndex,
    bool reverseCosts,
    bool log)
  :
    Search(
      graph,
      controller,
      startIndex,
      endIndex,
      reverseCosts,
      log)
  {
  }

  double getNodeSortValue(const std::shared_ptr<SearchNode> &node) override;

private:
};
