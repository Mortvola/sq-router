#include "Graph.h"
#include "Cost.h"
#include "./Database/DBConnection.h"
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <tuple>
#include <mutex>
#include <shared_mutex>
#include <optional>
#include <jsoncpp/json/json.h>

constexpr int Graph::NodeIdNone;


Graph::Graph ()
:
  m_loadNodes("loadNodes"),
  m_dbConnection(std::make_shared<DBConnection>()),
  m_queryNode(
    m_dbConnection,
    R"%(
      select n.id, ST_AsGeoJSON(ST_Transform(n.way, 4326)) AS point 
      from nav_nodes n 
      where id = $1 
    )%"),
  m_queryArea(
    m_dbConnection,
    R"%(
      select
        n.id AS node_id,
        e.line_id,
        e.fraction,
        ST_AsGeoJSON(ST_Transform(n.way, 4326)) AS point,
        e.id AS edge_id,
        e.point_index,
        COALESCE(e.forward_edge_id, -1) as forward_edge_id,
        COALESCE(fwe.node_id, -1) as forward_node_id,
        COALESCE(fwe.fraction, -1) as forward_fraction,
        COALESCE(fwe.reverse_cost, -1) as forward_reverse_cost,
        COALESCE(e.reverse_edge_id, -1) as reverse_edge_id,
        COALESCE(rve.node_id, -1) as reverse_node_id,
        COALESCE(rve.fraction, -1) as reverse_fraction,
        COALESCE(rve.forward_cost, -1) as reverse_forward_cost,
        e.forward_cost,
        e.reverse_cost
      from nav_nodes n 
      join nav_edges e on e.node_id = n.id 
      left join nav_edges fwe on fwe.id = e.forward_edge_id
      left join nav_edges rve on rve.id = e.reverse_edge_id
      where ST_Contains( 
        ST_SetSRID(ST_MakeBox2D( 
          ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), 
          ST_Transform(ST_SetSRID(ST_MakePoint($1 + 0.125, $2 + 0.125), 4326), 3857) 
        ), 3857), 
        n.way) 
      order by n.id, line_id, fraction 
    )%")
{
}

double Graph::distanceBetweenNodes(const Node &node1, const Node &node2)
{
	return haversineGreatCircleDistance(node1.m_point, node2.m_point);
}

double Graph::costBetweenNodes(const Node &node1, const Node &node2)
{
	auto distance = distanceBetweenNodes(node1, node2);

	if (distance == 0)
	{
		return 0;
	}

	return distance / metersPerHourGet(0, distance);
}


void Graph::loadNode(int nodeId)
{
	std::unique_lock<std::mutex> lock(m_graphAccess);

  loadNodeNoLock(nodeId);
}


void Graph::loadNodeNoLock(int nodeId)
{
	// Load the node if the node index is valid and it isn't already loaded

  try
  {
    if (nodeId != Graph::NodeIdNone)
    {
      m_loadNodes.start();

      auto i = m_nodes.find(nodeId);

      if (i == m_nodes.end())
      {
        auto start = std::chrono::steady_clock::now ();

        auto node = m_queryNode.exec1(nodeId);

        Json::Reader reader;
        Json::Value root;
        reader.parse(node["point"].as<std::string>(), root);

        double lat = root["coordinates"][1].asDouble();
        double lng = root["coordinates"][0].asDouble();
        lat = std::floor(lat * 8) / 8.0;
        lng = std::floor(lng * 8) / 8.0;

        auto nodes = m_queryArea.exec(lng, lat);

        int nodeId = -1;
        std::shared_ptr<Node> newNode;

        for (const auto &entry: nodes)
        {
          auto id = entry["node_id"].as<int>();

          // The node will be repeated in each record of each leg associated with the node
          // So, only create a new node when the node ID changes.
          if (id != nodeId)
          {
            nodeId = id;
            newNode = nullptr;

            if (m_nodes.find(nodeId) == m_nodes.end())
            {
              newNode = std::make_shared<Node>(nodeId);

              reader.parse(entry["point"].as<std::string>(), root);

              newNode->m_point = {root["coordinates"][1].asDouble(),
                  root["coordinates"][0].asDouble()};
              newNode->m_point.m_elevation = root["coordinates"][2].asDouble();

              m_nodes[newNode->getNodeId ()] = newNode;
            }
          }

          // Only process the edge information if we added a new node.
          if (newNode)
          {
            const std::string costColumns[]{"forward_cost", "reverse_cost"};
            const std::string otherCostColumns[]{"forward_reverse_cost", "reverse_forward_cost"};
            const std::string edgeIdColumns[]{"forward_edge_id", "reverse_edge_id"};
            const std::string nodeIdColumns[]{"forward_node_id", "reverse_node_id"};
            const std::string fractionColumns[]{"forward_fraction", "reverse_fraction"};

            auto lineId = entry["line_id"].as<int>();

            NavEdge navEdge;

            navEdge.m_edgeId = entry["edge_id"].as<int>();
            navEdge.m_nodeId = nodeId;
            navEdge.m_otherEdgeId[Edge::EndType::Start] = entry[edgeIdColumns[Edge::EndType::Start]].as<int>();
            navEdge.m_otherEdgeId[Edge::EndType::End] = entry[edgeIdColumns[Edge::EndType::End]].as<int>();

            for (auto endType: {Edge::EndType::Start, Edge::EndType::End})
            {
              if (entry[edgeIdColumns[endType]].as<int>() != -1)
              {
                auto iter = m_edges.find(entry[edgeIdColumns[endType]].as<int>());

                if (iter == m_edges.end())
                {
                  // The other node has not been loaded yet.
                  // Create and add edge to the currently loading node's edge list
                  auto edge = std::make_shared<Edge>();

                  //edge->m_edgeId = edgeId;
                  edge->m_pointIndex = entry["point_index"].as<int>();
                  edge->m_lineId = lineId;

                  edge->m_edgeEnd[endType].m_edgeId = entry["edge_id"].as<int>();
                  edge->m_edgeEnd[endType].m_nodeId = nodeId;
                  edge->m_edgeEnd[endType].m_node = newNode;
                  edge->m_edgeEnd[endType].m_fraction = entry["fraction"].as<double>();
                  edge->m_edgeEnd[endType].m_cost = entry[costColumns[endType]].as<double>();

                  edge->m_edgeEnd[endType ^ 1].m_edgeId = entry[edgeIdColumns[endType]].as<int>();
                  edge->m_edgeEnd[endType ^ 1].m_nodeId = entry[nodeIdColumns[endType]].as<int>();
                  edge->m_edgeEnd[endType ^ 1].m_node = nullptr;
                  edge->m_edgeEnd[endType ^ 1].m_fraction = entry[fractionColumns[endType]].as<double>();
                  edge->m_edgeEnd[endType ^ 1].m_cost = entry[otherCostColumns[endType]].as<double>();

                  newNode->addEdge(edge);

                  navEdge.m_edge[endType] = edge;
                }
                else
                {
                  // Other node was found. Get the edge from that node's edge list
                  auto otherEdge = iter->second;

                  auto edge = otherEdge.m_edge[endType ^ 1];

                  if (edge == nullptr)
                  {
                    throw std::runtime_error("found node but not the edge");
                  }

                  if (edge->m_edgeEnd[endType].m_nodeId != nodeId)
                  {
                    throw std::runtime_error("node id mismatch");
                  }
                
                  edge->m_edgeEnd[endType].m_node = newNode;
                  
                  newNode->addEdge(edge);            

                  navEdge.m_edge[endType] = edge;
                }
              }
            }

            m_edges[navEdge.m_edgeId] = navEdge;
          }
        }

        m_nodeLoadingTime += std::chrono::steady_clock::now () - start;
      }

      m_loadNodes.stop();
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
}

