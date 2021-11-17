#include "AStarSearch.h"

double AStarSearch::getNodeSortValue(const std::shared_ptr<SearchNode> &node)
{
  if (node == nullptr) {
    return 0;
  }

  return node->m_searchInfo[m_search].m_cummulativeCost
    + node->m_searchInfo[m_search].m_timeToEnd;
}
