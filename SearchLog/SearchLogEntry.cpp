#include "SearchLogEntry.h"
#include "SearchController.h"
#include "SearchNode.h"

void SearchLogEntry::addOpenNode(const std::shared_ptr<SearchNode> &node, int searchDirection)
{
  m_openNodes.push_back({
    node->getNodeId(),
    node->getLatLng(),
    node->getCumulativeCost(searchDirection),
    node->getTimeToEnd(searchDirection),
    node->m_preferredNode,
  });
}
