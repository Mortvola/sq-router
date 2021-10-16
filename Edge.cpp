/*
 * Edge.cpp
 *
 *  Created on: Oct 29, 2019
 *      Author: richard
 */
#include "Edge.h"
#include "Graph.h"


int Edge::getOtherNodeId(int nodeId) const
{
	if (m_edgeEnd[Edge::EndType::Start].m_nodeId != Graph::NodeIdNone
    && m_edgeEnd[Edge::EndType::Start].m_nodeId != nodeId)
	{
		return m_edgeEnd[Edge::EndType::Start].m_nodeId;
	}

	if (m_edgeEnd[Edge::EndType::End].m_nodeId != Graph::NodeIdNone
    && m_edgeEnd[Edge::EndType::End].m_nodeId != nodeId)
	{
		return m_edgeEnd[Edge::EndType::End].m_nodeId;
	}

	return Graph::NodeIdNone;
}

std::shared_ptr<Node> Edge::getOtherNode(const std::shared_ptr<Node> &node)
{
	if (m_edgeEnd[Edge::EndType::Start].m_node != node)
	{
		return m_edgeEnd[Edge::EndType::Start].m_node;
	}

	if (m_edgeEnd[Edge::EndType::End].m_node != node)
	{
		return m_edgeEnd[Edge::EndType::End].m_node;
	}

	return nullptr;
}

double Edge::getCost (int startNodeId, bool reverseCost) const
{
	if (reverseCost)
	{
		if (m_edgeEnd[Edge::EndType::Start].m_nodeId == startNodeId)
		{
			return m_edgeEnd[Edge::EndType::End].m_cost;
		}

		return m_edgeEnd[Edge::EndType::Start].m_cost;
	}

	if (m_edgeEnd[Edge::EndType::Start].m_nodeId == startNodeId)
	{
		return m_edgeEnd[Edge::EndType::Start].m_cost;
	}

	return m_edgeEnd[Edge::EndType::End].m_cost;
}
