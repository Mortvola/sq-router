#pragma once

#include "SearchLogNode.h"
#include "Point.h"
#include <vector>
#include <memory>

class SearchController;
class SearchNode;

class SearchLogEntry {
public:
  void addOpenNode(const std::shared_ptr<SearchNode> &node, int searchDirection);

  void removeOpenNode(int nodeId)
  {
    m_closedNodes.push_back(nodeId);
  }

  std::vector<SearchLogNode> m_openNodes;
  std::vector<int> m_closedNodes;

  operator Json::Value() const
  {
    Json::Value openNodes = Json::arrayValue;

    for (const auto &node: m_openNodes)
    {
      openNodes.append(node);
    }

    Json::Value closedNodes = Json::arrayValue;

    for (const auto &nodeId: m_closedNodes)
    {
      closedNodes.append(nodeId);
    }

    Json::Value value;

    value["openNodes"] = openNodes;
    value["closedNodes"] = closedNodes;

    return value;
  }
};
