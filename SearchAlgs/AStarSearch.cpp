#include "AStarSearch.h"

double AStarSearch::getNodeSortValue(const std::shared_ptr<SearchNode> &node)
{
  if (node == nullptr) {
    return 0;
  }

  return node->getCumulativeCost(m_searchDirection)
    + node->getTimeToEnd(m_searchDirection);
}
