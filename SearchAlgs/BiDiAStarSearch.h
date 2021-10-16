#pragma once

#include "Search.h"

class BiDiAStarSearch: public Search
{
public:
  BiDiAStarSearch(
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

  double getNodeSortCost(const std::shared_ptr<SearchNode> &node) override;

  double getPotentialPathCost(const std::shared_ptr<SearchNode> &node) override;

private:
};
