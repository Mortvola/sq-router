#include "SearchNode.h"
#include "SearchController.h"

std::shared_ptr<Edge> SearchNode::getEdgeOverride(const std::shared_ptr<Edge> &edge)
{
  auto iter = std::find_if(m_edgeOverrides.begin(), m_edgeOverrides.end(),
    [edge](const std::pair<std::shared_ptr<Edge>, std::shared_ptr<Edge>> &entry)
    {
      return edge == entry.first;
    }
  );

  if (iter != m_edgeOverrides.end())
  {
    return iter->second;
  }

  return edge;
}

std::shared_ptr<SearchEdge> SearchNode::createSearchEdge(const std::shared_ptr<Edge> &edge)
{
  auto tmpEdge = getEdgeOverride(edge);

  if (tmpEdge->m_edgeEnd[0].m_node == nullptr
    && tmpEdge->m_edgeEnd[0].m_nodeId != -1)
  {
    tmpEdge->m_edgeEnd[0].m_node = m_controller.m_graph->getNode(tmpEdge->m_edgeEnd[0].m_nodeId);
  }

  if (tmpEdge->m_edgeEnd[1].m_node == nullptr
    && tmpEdge->m_edgeEnd[1].m_nodeId != -1)
  {
    tmpEdge->m_edgeEnd[1].m_node = m_controller.m_graph->getNode(tmpEdge->m_edgeEnd[1].m_nodeId);
  }

  auto otherNode = tmpEdge->getOtherNode(m_node);

  if (otherNode)
  {
    auto otherSearchNode = m_controller.getSearchNode(otherNode);

    auto searchEdge = std::make_shared<SearchEdge>(tmpEdge);
  
    searchEdge->m_edgeNode[0] = shared_from_this();
    searchEdge->m_edgeNode[1] = otherSearchNode;

    auto lock1 = getUniqueSearchEdgeLock(std::defer_lock);
    auto lock2 = otherSearchNode->getUniqueSearchEdgeLock(std::defer_lock);

    // Always lock the node with the lower ID first
    // in order to prevent deadlocks.
    if (getNodeId() > otherSearchNode->getNodeId())
    {
      std::lock(lock2, lock1);
    }
    else
    {
      std::lock(lock1, lock2);
    }

    m_searchEdges.push_back(searchEdge);
    otherSearchNode->m_searchEdges.push_back(searchEdge);

    return searchEdge;
  }

  return nullptr;
}

std::shared_ptr<SearchEdge> SearchNode::getSearchEdge(const std::shared_ptr<Edge> &edge)
{
  std::shared_ptr<SearchEdge> searchEdge;

  {
    std::lock_guard<std::mutex> lock(m_searchEdgesAccess);

    searchEdge = getSearchEdgeNoLock(edge);
  }

  if (searchEdge == nullptr)
  {
    searchEdge = createSearchEdge(edge);
  }

  return searchEdge;
}

std::shared_ptr<SearchEdge> SearchNode::getSearchEdgeNoLock(const std::shared_ptr<Edge> &edge)
{
  auto iter = std::find_if(m_searchEdges.begin(), m_searchEdges.end(),
    [edge](const std::shared_ptr<SearchEdge> &searchEdge)
    {
      return searchEdge->m_edge == edge;
    }
  );

  if (iter == m_searchEdges.end())
  {
    return nullptr;
  }

  return *iter;
}

std::vector<std::future<std::shared_ptr<SearchNode>>> SearchNode::forEachEdge(
  ThreadPool &threadPool,
  std::function<std::shared_ptr<SearchNode>(const std::shared_ptr<Edge> &)> callback)
{
  return m_node->forEachEdge(
    threadPool,
    callback);
}

std::vector<std::shared_ptr<SearchEdge>> SearchNode::getSearchEdges()
{
  std::lock_guard<std::mutex> lock(m_searchEdgesAccess);

  return m_searchEdges;
}

std::shared_ptr<SearchEdge> SearchNode::getEntryEdge(int search)
{
  std::lock_guard<std::mutex> lock(m_searchEdgesAccess);

  auto iter = std::find_if(m_searchEdges.begin(), m_searchEdges.end(),
    [this, search](const std::shared_ptr<SearchEdge> &searchEdge)
    {
      return searchEdge->m_search == search
        && searchEdge->m_fromNodeId != -1
        && searchEdge->m_fromNodeId != this->getNodeId();
    });

  if (iter != m_searchEdges.end())
  {
    return *iter;
  }

  return nullptr;
}

bool SearchNode::hasNonTraversedEdges()
{
  std::lock_guard<std::mutex> lock(m_searchEdgesAccess);

  for (auto &edge: m_searchEdges)
  {
    if (edge->m_fromNodeId == -1)
    {
      return true;
    }
  }

  return false;
}

