/*
 *  Created on: Oct 29, 2019
 *      Author: richard
 */
#include "Search.h"
#include "SearchController.h"
#include "SearchLogEntry.h"
#include <algorithm>
#include <fstream>
#include <thread>
#include <future>

Search::Search(
  const std::shared_ptr<Graph> &graph,
  SearchController &controller,
  int startIndex,
  int endIndex,
  bool reverseCosts,
  bool log)
:
  m_searchDirection(reverseCosts ? SearchDirection::Reverse : SearchDirection::Forward),
  m_graph(graph),
  m_controller(controller),
  m_log(log),
  m_reverseCosts(reverseCosts)
{
  m_startNodeId = m_controller.createNode(startIndex);
  m_endNodeId = m_controller.createNode(endIndex);
}

void Search::initializeStartNode()
{
  auto startNode = m_controller.getSearchNode(m_startNodeId);
  auto endNode = m_controller.getSearchNode(m_endNodeId);

  startNode->m_searchInfo[m_searchDirection].m_cummulativeCost = 0;
#if 0
  if (m_reverseCosts)
  {
    startNode.m_searchInfo[m_searchDirection].m_costToEnd =
        m_graph->costBetweenNodes(*endNode.m_node, *startNode.m_node);
  }
  else
  {
    startNode.m_searchInfo[m_searchDirection].m_costToEnd =
        m_graph->costBetweenNodes(*startNode.m_node, *endNode.m_node);
  }
#endif

  setToEndValues(startNode);
  queueInsert(startNode);

  startNode->m_searchInfo[m_searchDirection].m_queued++;
}

double Search::getNodeSortValue(const std::shared_ptr<SearchNode> &node)
{
  return getNodeCumulativeCost(node);
}

double Search::getNodeCumulativeCost(const std::shared_ptr<SearchNode> &node)
{
  if (node == nullptr) {
    return 0;
  }

  return node->getCumulativeCost(m_searchDirection);
}

std::shared_ptr<SearchNode> Search::handleEdgeTraversal(
  const std::shared_ptr<SearchEdge> &searchEdge,
  const std::shared_ptr<SearchNode> &fromNode,
  const std::shared_ptr<SearchNode> &nextNode
)
{
  if (nextNode->getEntryEdge(m_searchDirection))
  {
    throw new std::runtime_error("has entry edge");
  }

  nextNode->m_fromNode = fromNode;

  searchEdge->m_fromNodeId = fromNode->getNodeId();
  searchEdge->m_searchDirection = m_searchDirection;

  checkSharedNode(nextNode);

  // Add the node to the queue if the cost to the end
  // is not known or the cost is less than the cost to the
  // end.
  if (m_controller.getBestCost() < 0 ||
      getNodeSortValue(nextNode) <= m_controller.getBestCost())
  {
    if (nextNode->m_searchInfo[m_searchDirection].m_queued)
    {
      std::cerr << "node already queued: "
                << nextNode->m_searchInfo[m_searchDirection].m_queued << "\n";
    }

    return nextNode;
  }

  return {};
}

std::shared_ptr<SearchNode> Search::handleBetterPath(
  const std::shared_ptr<SearchEdge> &searchEdge,
  const std::shared_ptr<SearchNode> &fromNode,
  const std::shared_ptr<SearchNode> &nextNode,
  double costDelta
)
{
  auto entryEdge = nextNode->getEntryEdge(m_searchDirection);

  if (entryEdge)
  {
    entryEdge->m_fromNodeId = -1;
    entryEdge->m_searchDirection = -1;

    // insertSearchNode(searchers, node, newSearcher);
  }
  else
  {
    throw std::runtime_error("found better path but no entry edge");
  }

  nextNode->m_fromNode = fromNode;

  {
    searchEdge->m_fromNodeId = fromNode->getNodeId();
    searchEdge->m_searchDirection = m_searchDirection;
  }

  checkSharedNode(nextNode);
  // propogateCostDelta(searchers, nextNode, costDelta);

  return {};
}

void Search::handleFoundEnd(
  const std::shared_ptr<SearchEdge> &searchEdge,
  const std::shared_ptr<SearchNode> &fromNode,
  const std::shared_ptr<SearchNode> &nextNode
)
{
  auto entryEdge = nextNode->getEntryEdge(m_searchDirection);

  if (entryEdge)
  {
    entryEdge->m_fromNodeId = -1;
    entryEdge->m_searchDirection = -1;
  }

  nextNode->m_fromNode = fromNode;

  {
    searchEdge->m_fromNodeId = fromNode->getNodeId();
    searchEdge->m_searchDirection = m_searchDirection;
  }

  m_controller.setBestCost(
    nextNode->m_searchInfo[m_searchDirection].m_cummulativeCost,
    nextNode->getNodeId());

  std::cerr << "Found end: cost: " << nextNode->m_searchInfo[m_searchDirection].m_cummulativeCost
            << ", bestCost: " << m_controller.getBestCost() << "\n";

  checkSharedNode(nextNode);
}

void Search::setToEndValues(const std::shared_ptr<SearchNode> &node)
{
  if (node->m_searchInfo[m_searchDirection].m_timeToEnd == -1)
  {
    auto distanceToEnd =
        m_graph->distanceBetweenNodes(
          *node->m_node,
          *m_controller.getSearchNode(m_endNodeId)->m_node
        );

    node->m_searchInfo[m_searchDirection].m_timeToEnd = distanceToEnd / 6000;
  }
}

std::shared_ptr<SearchNode> Search::getNeighbor(
  const std::shared_ptr<SearchEdge> &searchEdge,
  const std::shared_ptr<SearchNode> &fromNode)
{
  std::shared_ptr<SearchNode> newSearchNode;

  // Don't traverse the edge if it was already traversed
  if (searchEdge->m_fromNodeId == -1)
  {
    auto nextNode = searchEdge->getOtherNode(fromNode);

    if (nextNode)
    {
      m_profiler2.start();

      double newCummulativeCost{0};
      double edgeCost{0};

      edgeCost = searchEdge->getCost(fromNode->getNodeId());
      if (edgeCost <= 0)
      {
        throw std::runtime_error(
          "edge cost is less than or equal to zero. Line ID: "
          + std::to_string(searchEdge->getLineId())
          + ", Cost: " + std::to_string(edgeCost)
        );
      }

      if (searchEdge->isPreferred(m_controller)) {
        nextNode->m_preferredNode = true;
        newCummulativeCost = fromNode->m_searchInfo[m_searchDirection].m_cummulativeCost;
      }
      else {
        nextNode->m_preferredNode = false;
        newCummulativeCost = fromNode->m_searchInfo[m_searchDirection].m_cummulativeCost + edgeCost;
      }

      auto lock = nextNode->getUniqueLock();

      // The edgeCost must not be a negative value
      // Also, the next node's cost must not yet have been
      // set or the new cost must be less than the next node's current
      // cost.
      if (nextNode->m_searchInfo[m_searchDirection].m_cummulativeCost < 0
        || newCummulativeCost < nextNode->m_searchInfo[m_searchDirection].m_cummulativeCost)
      {
        nextNode->m_searchInfo[m_searchDirection].m_visitCount++;

        if (nextNode->getNodeId() == m_endNodeId)
        {
          // We have reached the target node.
          nextNode->setCumulativeCost(m_searchDirection, newCummulativeCost);
          nextNode->setTimeToEnd(m_searchDirection, m_graph, m_endNodeId);
          m_profiler3.start();
          handleFoundEnd(searchEdge, fromNode, nextNode);
          m_profiler3.stop();
        }
        else if (newCummulativeCost < nextNode->getCumulativeCost(m_searchDirection))
        {
          // The new cost is better than the previous cost to reach the next node
          auto costDelta = nextNode->m_searchInfo[m_searchDirection].m_cummulativeCost - newCummulativeCost;
          nextNode->setCumulativeCost(m_searchDirection, newCummulativeCost);
          nextNode->setTimeToEnd(m_searchDirection, m_graph, m_endNodeId);
          m_profiler4.start();
          newSearchNode = handleBetterPath(searchEdge, fromNode, nextNode, costDelta);
          m_profiler4.stop();
        }
        else
        {
          nextNode->setCumulativeCost(m_searchDirection, newCummulativeCost);
          nextNode->setTimeToEnd(m_searchDirection, m_graph, m_endNodeId);
          m_profiler5.start();
          newSearchNode = handleEdgeTraversal(searchEdge, fromNode, nextNode);
          m_profiler5.stop();
        }
      }

      m_profiler2.stop();
    }
  }

  return newSearchNode;
}

void Search::queueInsert(const std::shared_ptr<SearchNode> &searchNode) {
  std::unique_lock<std::mutex> lock(m_openSearchNodesMutex);
  m_openSearchNodes.insert(searchNode);
  searchNode->m_searchInfo[m_searchDirection].m_queued++;
}

void Search::insertOpenNode(
  const std::shared_ptr<SearchNode> &fromNode,
  const std::shared_ptr<SearchNode> &node)
{
  node->m_sortCost = getNodeSortValue(node);

  queueInsert(node);

  if (m_log)
  {
    m_controller.m_searchLog.addOpenNode(node, m_searchDirection);
  }
}

void Search::propogateCostDelta(
  const std::shared_ptr<SearchNode> &node,
  double costDelta)
{
  for (auto &edge: node->getSearchEdges())
  {
    if (edge->m_searchDirection == m_searchDirection && edge->m_fromNodeId == node->getNodeId())
    {
      auto nextNodeId = edge->getOtherNodeId(node->getNodeId());

      // bool searchNodeExists {false};

      // {
      //   searchNodeExists = nextNodeId != Graph::NodeIdNone
      //     && m_controller.m_openSearchNodes.find(nextNodeId) !=
      //     m_controller.m_openSearchNodes.end();
      // }

      // if (searchNodeExists)
      {
        auto nextNode = m_controller.getSearchNode(nextNodeId);

        if (nextNode->m_searchInfo[m_searchDirection].m_cummulativeCost != -1)
        {
          if (nextNode->m_searchInfo[m_searchDirection].m_cummulativeCost < costDelta)
          {
            throw std::runtime_error("cost underflow");
          }

          nextNode->m_searchInfo[m_searchDirection].m_cummulativeCost -= costDelta;
          checkSharedNode(nextNode);
          propogateCostDelta(nextNode, costDelta);

          // If this node has an non-traversed edge (which may occur if
          // the node at the time was blocked by too high of cost to
          // traverse the edge) then add this node to the search nodes.
          if (nextNode->hasNonTraversedEdges())
          {
            insertOpenNode(node, nextNode);
          }
        }
      }
    }
  }
}

double Search::getNodeCost(int nodeId)
{
  return m_controller.getNodeCost(nodeId, m_searchDirection);
}

bool Search::isSharedNode(const std::shared_ptr<SearchNode> &node)
{
  auto otherSearch = m_otherSearch.lock();
  return otherSearch && node->m_searchInfo[m_searchDirection ^ 1].m_cummulativeCost >= 0;
}

void Search::checkSharedNode(const std::shared_ptr<SearchNode> &node)
{
  if (isSharedNode(node))
  {
    auto cost = node->m_searchInfo[m_searchDirection].m_cummulativeCost
      + node->m_searchInfo[m_searchDirection ^ 1].m_cummulativeCost;

    m_controller.setBestCost(cost, node->getNodeId());
  }
}

void Search::processNext(ThreadPool &threadPool)
{
  if (m_log)
  {
    m_controller.m_searchLog.startNewEntry();
  }

  std::shared_ptr<SearchNode> searchNode;

  m_profileProcessNext.start();

  searchNode = queuePopFront();

  searchNode->m_searchInfo[m_searchDirection].m_queued--;

  m_profiler8.start();
  searchNode->forEachEdge(
    [this, searchNode](const std::shared_ptr<Edge> &edge)
    {
      auto searchEdge = searchNode->getSearchEdge(edge);

      if (searchEdge != nullptr)
      {
        auto neighbor = getNeighbor(searchEdge, searchNode);

        if (neighbor) {
          insertOpenNode(searchNode, neighbor);
        }
      }
    }
  );
  m_profiler8.stop();

  if (m_log)
  {
    m_controller.m_searchLog.removeOpenNode(searchNode->getNodeId());
  }

  m_profileProcessNext.stop();
}
