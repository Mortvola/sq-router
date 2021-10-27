#include "BiDiAStarSearch.h"

double BiDiAStarSearch::getNodeSortCost(const std::shared_ptr<SearchNode> &node)
{
  if (node == nullptr) {
    return 0;
  }

  return getPotentialPathCost(node);
}

double BiDiAStarSearch::getPotentialPathCost(const std::shared_ptr<SearchNode> &node)
{
  return node->m_searchInfo[m_search].m_cummulativeCost
    + node->m_searchInfo[m_search].m_timeToEnd;
}
