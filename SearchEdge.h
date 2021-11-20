/*
 * SearchEdge.h
 *
 *  Created on: Oct 29, 2019
 *      Author: richard
 */

#pragma once

#include "Edge.h"
#include "SearchController.h"
#include <memory>

class SearchNode;

class SearchEdge: public std::enable_shared_from_this<SearchEdge>
{
public:
  explicit SearchEdge(const std::shared_ptr<Edge> &edge)
  :
    m_edge(edge)
  {}

  int getStartNode() { return m_edge->m_edgeEnd[Edge::EndType::Start].m_nodeId; }

  int getEndNode() { return m_edge->m_edgeEnd[Edge::EndType::End].m_nodeId; }

  double getForwardCost() {
    if (m_edge->m_edgeEnd[Edge::EndType::Start].m_cost <= 0) {
      throw std::runtime_error("Cost is negative");
    }

    return m_edge->m_edgeEnd[Edge::EndType::Start].m_cost;
  }

  double getBackwardCost() {
    if (m_edge->m_edgeEnd[Edge::EndType::End].m_cost <= 0) {
      throw std::runtime_error("Cost is negative");
    }

    return m_edge->m_edgeEnd[Edge::EndType::End].m_cost;
  }

  double getCost(int startNodeId) {
    return m_edge->getCost(startNodeId, m_searchDirection);
  }

  int getOtherNodeId(int nodeId)
  {
    return m_edge->getOtherNodeId(nodeId);
  }

  std::shared_ptr<SearchNode> getOtherNode(const std::shared_ptr<SearchNode> &otherNode)
  {
    if (otherNode == m_edgeNode[0])
    {
      return m_edgeNode[1];
    }

    if (otherNode == m_edgeNode[1])
    {
      return m_edgeNode[0];
    }
  
    return nullptr;
  }

  std::shared_ptr<Edge> getEdge()
  {
    return m_edge;
  }

  int getLineId() { return m_edge->m_lineId; }

  double getStartFraction() { return m_edge->m_edgeEnd[Edge::EndType::Start].m_fraction; }

  double getEndFraction() { return m_edge->m_edgeEnd[Edge::EndType::End].m_fraction; }

  friend std::ostream &operator<<(std::ostream &os, const SearchEdge &edge) {
    os << *edge.m_edge;

    return os;
  }

  bool isPreferred(SearchController &controller)
  {
    if (m_preferred == PreferredEdge::Unknown) {
      m_preferred = controller.isEdgePreferred(
      m_edge->m_edgeEnd[0].m_edgeId,
      m_edge->m_edgeEnd[1].m_edgeId)
      ? PreferredEdge::True
      : PreferredEdge::False;
    }

    return m_preferred;
  }

  int m_searchDirection{-1};
  int m_fromNodeId{-1};
  std::shared_ptr<Edge> m_edge;

  std::shared_ptr<SearchNode> m_edgeNode[2];

private:

  enum PreferredEdge 
  {
    Unknown = -1,
    False = 0,
    True = 1
  };

  PreferredEdge m_preferred{PreferredEdge::Unknown};
};
