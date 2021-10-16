#include "AStarSearch.h"

double AStarSearch::getNodeSortCost(const std::shared_ptr<SearchNode> &node)
{
  return node->m_searchInfo[m_search].m_cummulativeCost
    + node->m_searchInfo[m_search].m_timeToEnd;
}
