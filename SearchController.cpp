#include "SearchController.h"
#include "SplitEdge.h"
#include "Search.h"
#include "AStarSearch.h"
#include "BiDiAStarSearch.h"
#include "StatusUpdate.h"
#include <thread>

SearchController::SearchController(
  const std::shared_ptr<Graph> &graph,
  const std::string &algorithm,
  const Point &point1,
  const Point &point2,
  bool returnLog)
:
  m_graph(graph),
  m_searchLog(*this),
  m_algorithm(algorithm),
  m_points{point1, point2},
  m_returnLog(returnLog)
{
  loadRouteGroup("test");
}

std::vector<std::vector<Anchor>> SearchController::search(ThreadPool &threadPool)
{
  for (size_t j = 0; j < 2; j++)
  {
    auto trailInfo = getTrailFromPoint(m_points[j]);

    if (trailInfo.m_startEdgeId == -1 && trailInfo.m_endEdgeId == -1)
    {
      throw std::runtime_error("No nav-edge found");
    }

    insertNode(trailInfo);
  }

  overrideEdges();

  std::vector<std::vector<Anchor>> anchors;

  Json::Value status = Json::objectValue;
  status["status"] = StatusSearchStarted;
  if (m_algorithm == "BiDijkstra")
  {
    status["algorithm"] = "BiDijkstra";
    statusUpdate(status);
    anchors.push_back(BiDiDijkstra(threadPool));
  }
  else if (m_algorithm == "A*")
  {
    status["algorithm"] = "A*";
    statusUpdate(status);
    anchors.push_back(AStar(threadPool));
  }
  else if (m_algorithm == "BiDiA*")
  {
    status["algorithm"] = "BiDiA*";
    statusUpdate(status);
    anchors.push_back(BiDiAStar(threadPool));
  }
  else
  {
    throw std::runtime_error("Unknown search algorithm");
  }

  return anchors; 
}

std::vector<Anchor> SearchController::BiDiDijkstra(ThreadPool &threadPool)
{
  auto forwardSearch = std::make_shared<Search>(m_graph, *this, 0, 1, false, m_returnLog);
  auto backwardSearch = std::make_shared<Search>(m_graph, *this, 1, 0, true, m_returnLog);

  forwardSearch->setOtherSearch(backwardSearch);
  backwardSearch->setOtherSearch(forwardSearch);

  forwardSearch->initializeStartNode();
  backwardSearch->initializeStartNode();

  try
  {
    while(forwardSearch->queueSize() > 0 && backwardSearch->queueSize() > 0)
    {
      if (forwardSearch->queueSize() > backwardSearch->queueSize()
       && backwardSearch->queueSize() > 0)
      {
        backwardSearch->processNext(threadPool);
      }
      else
      {
        forwardSearch->processNext(threadPool);
      }

      if (getBestCost() >= 0
       && forwardSearch->getNodeSortCost(forwardSearch->queueFront())
       + backwardSearch->getNodeSortCost(backwardSearch->queueFront())
       >= getBestCost())
      {
        break;
      }
    }

    Json::Value status = Json::objectValue;
    status["status"] = StatusSearchCompleted;
    statusUpdate(status);

    return getRoute(forwardSearch, backwardSearch);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }

  return {};
}

std::vector<Anchor>  SearchController::AStar(ThreadPool &threadPool)
{
  auto search = std::make_shared<AStarSearch>(m_graph, *this, 0, 1, false, m_returnLog);

  search->initializeStartNode();

  try
  {
    while(search->queueSize() > 0)
    {
      search->processNext(threadPool);
    }

    return getRoute(search, nullptr);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }

  return {};
}

std::vector<Anchor> SearchController::BiDiAStar(ThreadPool &threadPool)
{
  auto forwardSearch = std::make_shared<BiDiAStarSearch>(m_graph, *this, 0, 1, false, m_returnLog);
  auto backwardSearch = std::make_shared<BiDiAStarSearch>(m_graph, *this, 1, 0, true, m_returnLog);

  forwardSearch->setOtherSearch(backwardSearch);
  backwardSearch->setOtherSearch(forwardSearch);

  forwardSearch->initializeStartNode();
  backwardSearch->initializeStartNode();

  try
  {
    m_searchProfiler.start();

    while(forwardSearch->queueSize() > 0 && backwardSearch->queueSize() > 0)
    {
#if 1
      bool searchSelection = forwardSearch->queueSize() > backwardSearch->queueSize()
       && backwardSearch->queueSize() > 0;

      if (searchSelection)
      {
        backwardSearch->processNext(threadPool);
      }
      else
      {
        forwardSearch->processNext(threadPool);
      }

      bool finished = getBestCost() >= 0
       && forwardSearch->getNodeSortCost(forwardSearch->queueFront())
       + backwardSearch->getNodeSortCost(backwardSearch->queueFront())
       >= getBestCost();

      if (finished)
      {
        break;
      }
#else
      auto &forwardNode = forwardSearch->queue().front().m_searchNode;
      auto &backwardNode = backwardSearch->queue().front().m_searchNode;
      
      auto c = std::min(
        forwardSearch->getNodeSortCost(forwardNode),
        backwardSearch->getNodeSortCost(backwardNode)
      );

      if (getBestCost() >= 0 && getBestCost() <= std::max(
        c,
        std::max(
          std::max(
            forwardSearch->getPotentialPathCost(forwardNode),
            backwardSearch->getPotentialPathCost(backwardNode)),
          std::max(
            forwardNode.m_searchInfo[0].m_cummulativeCost,
            backwardNode.m_searchInfo[1].m_cummulativeCost))))
      {
        break;
      }

      if (c == forwardSearch->getNodeSortCost(forwardNode) && forwardSearch->queue().size() > 0)
      {
        forwardSearch->processNext(threadPool);
      }
      else
      {
        backwardSearch->processNext(threadPool);
      }
#endif
    }

    m_searchProfiler.stop();

    return getRoute(forwardSearch, backwardSearch);
  }
  catch (const std::exception &e)
  {
    std::cerr << e.what() << '\n';
  }

  return {};
}

std::shared_ptr<SearchNode> SearchController::getSearchNode(int nodeId)
{
  std::lock_guard<std::mutex> lock(m_searchNodesAccess);

  auto iter = m_searchNodes.find(nodeId);

  if (iter == m_searchNodes.end())
  {
    std::shared_ptr<Node> node;

    if (nodeId < 0)
    {
      auto iter = std::find_if(m_nodes.begin(), m_nodes.end(),
        [nodeId](const std::shared_ptr<Node> &node)
        {
          return node->getNodeId() == nodeId;
        }
      );

      if (iter == m_nodes.end())
      {
        throw std::runtime_error("node not found");
      }

      node = *iter;
    }
    else
    {
      m_getNodeProfiler.start();
      node = m_graph->getNode(nodeId);
      m_getNodeProfiler.stop();
    }

    return m_searchNodes
        .emplace(nodeId, std::make_shared<SearchNode>(*this, node))
        .first->second;
  }

  return iter->second;
}

std::shared_ptr<SearchNode> SearchController::getSearchNode(const std::shared_ptr<Node> &node)
{
  m_getNodeProfiler.start();

  std::lock_guard<std::mutex> lock(m_searchNodesAccess);

  auto iter = m_searchNodes.find(node->getNodeId());

  if (iter == m_searchNodes.end())
  {
    m_getNodeProfiler.stop();
    
    return m_searchNodes
        .emplace(node->getNodeId(), std::make_shared<SearchNode>(*this, node))
        .first->second;
  }

  m_getNodeProfiler.stop();

  return iter->second;
}

double SearchController::getBestCost() const
{
  return bestCost;
}

void SearchController::setBestCost(double cost, int nodeId)
{
  if (bestCost < 0 || cost < bestCost)
  {
    bestCost = cost;
    m_bestCostNodeId = nodeId;
  }
}

int SearchController::insertNode(TrailInfo &terminus)
{
  if (terminus.m_point.m_elevation == -1)
  {
    terminus.m_point.m_elevation =
        Elevation::getInstance()->getElevation(terminus.m_point);
  }

  if (terminus.m_startFraction != terminus.m_fraction)
  {
    auto nodeId = m_insertedNodeSeq--;

    auto newNode = std::make_shared<Node>(nodeId);

    newNode->m_point = terminus.m_point;

  //  auto originalEdgeId = terminus.m_startEdgeId;

    std::shared_ptr<SplitEdge> splitEdge;
    int startNodeId {-1};
    double startFraction {-1};
    int endNodeId {-1};
    double endFraction {-1};

    std::vector<std::pair<double, std::shared_ptr<Node>>>::iterator newNodeIter;

    // Determine if this edge is already split. If so,
    // add the new node to the list. If no, create a split edge
    // and add the new node to it.
    auto iter = std::find_if(m_splits.begin(), m_splits.end(),
      [&terminus](const std::shared_ptr<SplitEdge> &split)
      {
        return split->m_lineId == terminus.m_lineId
          && split->m_startFraction == terminus.m_startFraction
          && split->m_endFraction == terminus.m_endFraction;
      }
    );

    if (iter == m_splits.end())
    {
      splitEdge = std::make_shared<SplitEdge>();

      splitEdge->m_lineId = terminus.m_lineId;
      splitEdge->m_startFraction = terminus.m_startFraction;
      splitEdge->m_endFraction = terminus.m_endFraction;

      splitEdge->m_nodes.push_back({terminus.m_fraction, newNode});
      newNodeIter = splitEdge->m_nodes.begin();

      m_splits.push_back(splitEdge);

      startNodeId = terminus.m_startNodeId;
      startFraction = terminus.m_startFraction;
      endNodeId = terminus.m_endNodeId;
      endFraction = terminus.m_endFraction;
    }
    else
    {
      splitEdge = *iter;

      // Determine where this new node lies in relation to the
      // existing nodes on the split edge.

      auto iter = std::find_if(splitEdge->m_nodes.begin(), splitEdge->m_nodes.end(), 
        [&terminus](std::pair<double, std::shared_ptr<Node>> split)
        {
          return split.first > terminus.m_fraction;
        }
      );
        
      newNodeIter = splitEdge->m_nodes.insert(iter, {terminus.m_fraction, newNode});

      if (newNodeIter == splitEdge->m_nodes.begin())
      {
        startNodeId = terminus.m_startNodeId;
        startFraction = terminus.m_startFraction;
      }
      else
      {
        startNodeId = (newNodeIter - 1)->second->getNodeId();
        startFraction = (newNodeIter - 1)->first;
      }

      if ((newNodeIter + 1) == splitEdge->m_nodes.end())
      {
        endNodeId = terminus.m_endNodeId;
        endFraction = terminus.m_endFraction;
      }
      else
      {
        endNodeId = (newNodeIter + 1)->second->getNodeId(); 
        endFraction = (newNodeIter + 1)->first;
      }
    }

    double forwardCost, backwardCost;

    if (newNodeIter == splitEdge->m_nodes.begin())
    {
      std::tie(forwardCost, backwardCost) = computeLineSubstringCost(
          terminus.m_lineId, startFraction, terminus.m_fraction);

      auto edge0 = std::make_shared<Edge>();

      edge0->m_edgeEnd[Edge::EndType::Start].m_nodeId = startNodeId;
      edge0->m_edgeEnd[Edge::EndType::Start].m_node = m_graph->getNode(startNodeId);
      edge0->m_edgeEnd[Edge::EndType::Start].m_fraction = startFraction;
      edge0->m_edgeEnd[Edge::EndType::Start].m_cost = forwardCost;
      edge0->m_edgeEnd[Edge::EndType::End].m_nodeId = nodeId;
      edge0->m_edgeEnd[Edge::EndType::End].m_node = newNode;
      edge0->m_edgeEnd[Edge::EndType::End].m_fraction = terminus.m_fraction;
      edge0->m_edgeEnd[Edge::EndType::End].m_cost = backwardCost;
      edge0->m_lineId = terminus.m_lineId;
      // edge0->m_edgeId = m_splitEdgeSeq--;
      // edge0->m_originalStartEdgeId = originalEdgeId;

      newNode->addEdge(edge0);
    }
    else
    {
      // There is a node on this split line before the new node. We need to fixup it's edge to point to the new node.
      auto previousNode = (newNodeIter - 1)->second;
      auto edge = previousNode->edges()[1];
      edge->m_edgeEnd[Edge::EndType::End].m_nodeId = newNode->getNodeId();
      edge->m_edgeEnd[Edge::EndType::End].m_fraction = terminus.m_fraction;

      newNode->addEdge(edge);
    }

    // if (terminus.m_startNodeId != Graph::NodeIdNone)
    // {
    //   auto i = m_nodes.find(terminus.m_startNodeId);

    //   if (i != m_nodes.end())
    //   {
    //     auto &node = i->second;

    //     node->substituteEdge(terminus.m_edgeId, edge0);
    //   }
    // }

    if (endNodeId != -1) {
      if ((newNodeIter + 1) == splitEdge->m_nodes.end())
      {
        std::tie(forwardCost, backwardCost) = computeLineSubstringCost(
            terminus.m_lineId, terminus.m_fraction, endFraction);

        auto edge1 = std::make_shared<Edge>();

        edge1->m_edgeEnd[Edge::EndType::Start].m_nodeId = nodeId;
        edge1->m_edgeEnd[Edge::EndType::Start].m_node = newNode;
        edge1->m_edgeEnd[Edge::EndType::Start].m_fraction = terminus.m_fraction;
        edge1->m_edgeEnd[Edge::EndType::Start].m_cost = forwardCost;
        edge1->m_edgeEnd[Edge::EndType::End].m_nodeId = endNodeId;
        edge1->m_edgeEnd[Edge::EndType::End].m_node = m_graph->getNode(endNodeId);
        edge1->m_edgeEnd[Edge::EndType::End].m_fraction = endFraction;
        edge1->m_edgeEnd[Edge::EndType::End].m_cost = backwardCost;
        edge1->m_lineId = terminus.m_lineId;
        // edge1->m_edgeId = m_splitEdgeSeq--;
        // edge1->m_originalStartEdgeId = originalEdgeId;

        newNode->addEdge(edge1);
      }
      else
      {
        // There is a node on this split line after the new node. We need to fixup it's edge to point to the new node.
        auto nextNode = (newNodeIter + 1)->second;
        auto edge = nextNode->edges()[0];
        edge->m_edgeEnd[Edge::EndType::Start].m_nodeId = newNode->getNodeId();
        edge->m_edgeEnd[Edge::EndType::Start].m_fraction = terminus.m_fraction;

        newNode->addEdge(edge);
      }
    }

    m_nodes.push_back(newNode);

    return nodeId;
  }

  // if (terminus.m_endNode != Graph::NodeIdNone)
  // {
  //   auto i = m_nodes.find(terminus.m_endNode);

  //   if (i != m_nodes.end())
  //   {
  //     auto &node = i->second;

  //     node->substituteEdge(terminus.m_edgeId, edge1);
  //   }
  // }

  m_nodes.push_back(m_graph->getNode(terminus.m_startNodeId));

  return terminus.m_startNodeId;
}

void SearchController::overrideEdges()
{
  for (const auto &split: m_splits)
  {
    auto &firstNode = split->m_nodes[0];
    auto &lastNode = split->m_nodes[split->m_nodes.size() - 1];

    int startNodeId = firstNode.second->edges()[0]->m_edgeEnd[0].m_nodeId;
    int endNodeId = lastNode.second->edges()[1]->m_edgeEnd[1].m_nodeId;

    auto startNode = getSearchNode(startNodeId);
    auto overriddenEdge = startNode->getEdge(endNodeId, split->m_lineId);
    startNode->m_edgeOverrides.push_back({overriddenEdge, firstNode.second->edges()[0]});

    auto endNode = getSearchNode(endNodeId);
    overriddenEdge = endNode->getEdge(startNodeId, split->m_lineId);
    endNode->m_edgeOverrides.push_back({overriddenEdge, lastNode.second->edges()[1]});
  }
}

void SearchController::loadRouteGroup(const std::string &name)
{
#if 0
  PreparedStatement queryGroup(
    R"%(
      select line_id
      from route_groups as rg
      join route_group_routes as rgr on rgr.route_group_id = rg.id
      where name = $1;
    )%"
  );

  auto result = queryGroup.exec(name);

  for (const auto &row: result)
  {
    m_preferredRouteGroupIds.push_back(row["line_id"].as<int>());
  }
#endif
}

int SearchController::createNode(int index)
{
  std::lock_guard<std::mutex> lock(m_searchNodesAccess);

  auto &node = m_nodes[index];

  auto nodeIter = m_searchNodes.find(node->getNodeId());

  if (nodeIter == m_searchNodes.end())
  {
    m_searchNodes
      .emplace(node->getNodeId(), std::make_shared<SearchNode>(*this, node))
      .first->second;
  }

  return node->getNodeId();
}

double SearchController::getNodeCost(int nodeId, int search)
{
  std::lock_guard<std::mutex> lock(m_searchNodesAccess);

  auto iter = m_searchNodes.find(nodeId);

  if (iter != m_searchNodes.end())
  {
    return iter->second->m_searchInfo[search].m_cummulativeCost;
  }

  return -1;
}

Anchor &getLastAnchor(std::vector<Anchor> &anchors, int search) {
  if (search) {
    return anchors[anchors.size() - 1];
  }

  return anchors[0];
}

Anchor &getPreviousAdded(std::vector<Anchor> &anchors, int search) {
  if (search) {
    return anchors[anchors.size() - 1];
  }

  return anchors[0];
}

Anchor &getNextToPreviousAdded(std::vector<Anchor> &anchors, int search) {
  if (search) {
    return anchors[anchors.size() - 2];
  }

  return anchors[1];
}

void addAnchor(std::vector<Anchor> &anchors, const Anchor &anchor, int search) {
  if (search)
  {
    anchors.insert(anchors.end(), anchor);
  }
  else
  {
    anchors.insert(anchors.begin(), anchor);
  }
}

void SearchController::addNewAnchor(
  std::vector<Anchor> &newAnchors,
  Anchor &anchor,
  int direction)
{
  if (newAnchors.size() > 0) {
    auto &prevAdded = getPreviousAdded(newAnchors, direction);

    if (anchor.m_point.m_lat != prevAdded.m_point.m_lat ||
        anchor.m_point.m_lng != prevAdded.m_point.m_lng) {
      if (anchor.m_lineEndpoint[direction ^ 1].m_lineId !=
          prevAdded.m_lineEndpoint[direction].m_lineId) {
        std::cerr << "next and previous trails don't match: "
                  << anchor.m_lineEndpoint[direction ^ 1].m_lineId << ", "
                  << prevAdded.m_lineEndpoint[direction].m_lineId << std::endl;

        throw std::runtime_error("next and previous trails don't match");
      }

      // Determine if we should replace the previously pushed anchor
      // or add a new one. If the anchor at position 0 is between
      // the new one and the one at position 1, then we can just replace
      // the one at position 0.
      if (newAnchors.size() > 1) {
        auto &nextToPrevAdded = getNextToPreviousAdded(newAnchors, direction);

        if (anchor.m_lineEndpoint[direction ^ 1].m_lineId ==
                prevAdded.m_lineEndpoint[direction].m_lineId &&
            anchor.m_lineEndpoint[direction ^ 1].m_lineId ==
                nextToPrevAdded.m_lineEndpoint[direction].m_lineId &&
            ((anchor.m_lineEndpoint[direction ^ 1].m_fraction >
                  prevAdded.m_lineEndpoint[direction].m_fraction &&
              prevAdded.m_lineEndpoint[direction ^ 1].m_fraction >
                  nextToPrevAdded.m_lineEndpoint[direction].m_fraction) ||
             (anchor.m_lineEndpoint[direction ^ 1].m_fraction <
                  prevAdded.m_lineEndpoint[direction].m_fraction &&
              prevAdded.m_lineEndpoint[direction ^ 1].m_fraction <
                  nextToPrevAdded.m_lineEndpoint[direction].m_fraction))) {
          prevAdded = anchor;
        } else {
          addAnchor(newAnchors, anchor, direction);
        }
      } else {
        addAnchor(newAnchors, anchor, direction);
      }
    }
  } else {
    newAnchors.push_back(anchor);
  }
}

std::vector<Anchor> SearchController::generateAnchors(
  int startNodeId,
  int endNodeId,
  int direction)
{
  std::vector<Anchor> newAnchors;

  int nodeId{endNodeId};
  std::shared_ptr<SearchEdge> prevEdge;

  do {
    auto nodeIter = m_searchNodes.find(nodeId);

    if (nodeIter == m_searchNodes.end())
    {
      break;
    }

    auto &node = nodeIter->second;
    auto lock = node->getUniqueLock();

    Anchor anchor;

    anchor.m_point = node->getPoint();

    // todo: do we need to assign types here?
    if (nodeId == startNodeId) {
      if (direction == 0) {
        anchor.m_type = "start";
      } else {
        anchor.m_type = "end";
      }
    } else if (nodeId == endNodeId) {
      if (direction == 0) {
        anchor.m_type = "end";
      } else {
        anchor.m_type = "start";
      }
    }

    if (prevEdge) {
      anchor.m_lineEndpoint[direction ^ 1].m_lineId = prevEdge->getLineId();

      if (prevEdge->getStartNode() != Graph::NodeIdNone &&
          prevEdge->getStartNode() == nodeId) {
        anchor.m_lineEndpoint[direction ^ 1].m_fraction =
            prevEdge->getStartFraction();
      } else if (prevEdge->getEndNode() != Graph::NodeIdNone &&
                 prevEdge->getEndNode() == nodeId) {
        anchor.m_lineEndpoint[direction ^ 1].m_fraction =
            prevEdge->getEndFraction();
      }
    }

    prevEdge = node->getEntryEdge(direction);

    if (prevEdge) {
      // if (prevEdge->getDeadEnd())
      // {
      //   std::cerr << "path along deadend edge: " << prevEdge->getEdgeId() <<
      //   "\n";
      // }

      anchor.m_lineEndpoint[direction].m_lineId = prevEdge->getLineId();

      if (prevEdge->getStartNode() == nodeId) {
        anchor.m_lineEndpoint[direction].m_fraction =
            prevEdge->getStartFraction();
        nodeId = prevEdge->getEndNode();
      } else if (prevEdge->getEndNode() == nodeId) {
        anchor.m_lineEndpoint[direction].m_fraction =
            prevEdge->getEndFraction();
        nodeId = prevEdge->getStartNode();
      } else {
        throw std::runtime_error("no previous or next");
      }
    }

    addNewAnchor(newAnchors, anchor, direction);
  } while (prevEdge);

  // If we have less than two anchors or if the start
  // and end anchors were not tagged correctly then
  // assume there was an error and clear out all anchors.
  if (newAnchors.size() < 2 || newAnchors[0].m_type != "start" ||
      newAnchors[newAnchors.size() - 1].m_type != "end")
  {
    newAnchors.clear();
  }

  return newAnchors;
}

std::vector<Anchor> SearchController::getRoute(
  const std::shared_ptr<Search> &forwardSearch,
  const std::shared_ptr<Search> &backwardSearch)
{
  if (m_bestCostNodeId != -1)
  {
    auto anchors = generateAnchors(forwardSearch->getStartNodeId(), m_bestCostNodeId, 0);

    if (backwardSearch && m_bestCostNodeId != backwardSearch->getStartNodeId())
    {
      auto otherAnchors = generateAnchors(
          backwardSearch->getStartNodeId(), m_bestCostNodeId, 1);

      if (otherAnchors.size() > 0)
      {
        if (anchors.size() > 0)
        {
          auto anchor = anchors.back();
          anchors.pop_back();

          otherAnchors[0].m_lineEndpoint[0] = anchor.m_lineEndpoint[0];
          anchors.insert(anchors.end(), otherAnchors.begin(), otherAnchors.end());
        }
        else
        {
          anchors = otherAnchors;
        }
      }
    }

    if (anchors.size() > 0)
    {
      for (size_t i = 0; i < anchors.size() - 1; i++)
      {
        if (anchors[i].m_lineEndpoint[1].m_lineId == -1 ||
            anchors[i + 1].m_lineEndpoint[0].m_lineId == -1 ||
            anchors[i].m_lineEndpoint[1].m_lineId !=
                anchors[i + 1].m_lineEndpoint[0].m_lineId)
        {
          throw std::runtime_error("invalid route points");
        }
      }
    }

    return anchors;
  }

  return {};
}
