#include "BiDiAStarSearch.h"

double BiDiAStarSearch::getNodeSortValue(const std::shared_ptr<SearchNode> &node)
{
  if (node == nullptr) {
    return 0;
  }

  return node->m_searchInfo[m_searchDirection].m_cummulativeCost
    + node->m_searchInfo[m_searchDirection].m_timeToEnd;
}
