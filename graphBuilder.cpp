/*
 * graphBuilder.cpp
 *
 *  Created on: Dec 20, 2019
 *      Author: richard
 */
#include "graphBuilder.h"
#include <iostream>
#include <jsoncpp/json/json.h>
#include "configuration.h"
#include "DBConnection.h"
#include "Cost.h"
#include "Elevation.h"
#include "Profiler.h"
#include "StatusUpdate.h"
#include <string>
#include <vector>
#include <set>
#include <tuple>
#include <cmath>
#include <chrono>

namespace gb
{
GraphBuilder::GraphBuilder()
:
  m_dbConnection(std::make_shared<DBConnection>()),
  m_queryIntersections (
    m_dbConnection,
    R"%(
      select l1.line_id AS line_id, array_agg(l2.line_id) AS other_lines
      from planet_osm_route l1 
      left join planet_osm_route l2 on
        l1.way && l2.way
        AND ST_Intersects(l1.way, l2.way)
        AND l1.line_id != l2.line_id
      where l1.way && ST_SetSRID(ST_MakeBox2D(
        ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857),
        ST_Transform(ST_SetSRID(ST_MakePoint($3, $4), 4326), 3857)
      ), 3857)
      AND ST_Intersects(l1.way, ST_SetSRID(ST_MakeBox2D(
        ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857),
        ST_Transform(ST_SetSRID(ST_MakePoint($3, $4), 4326), 3857)
      ), 3857))
      group by l1.line_id
      order by l1.line_id
    )%"),
  m_queryIntersectionPoints (
    m_dbConnection,
    R"%(
      with lines as (
        select
          tmp.point,
          tmp.line_id,
          tmp.way2,
          (tmp.point).path[1] as index,
          case
            when (tmp.point).path[1] = 1 THEN 0 -- the first point on a line is always at fraction 0.0
            else
              sum(ST_Distance(
                ST_PointN(tmp.way2, (tmp.point).path[1] - 1), ST_PointN(tmp.way2, (tmp.point).path[1])))
                over (PARTITION BY tmp.line_id ORDER BY tmp.line_id, (tmp.point).path[1] asc rows between unbounded preceding and current row)
                / ST_Length(tmp.way2)
          end as fraction
        from (
          select 
            ST_DumpPoints(way2) AS point,
            line_id,
            way2
          from planet_osm_route
          where line_id = ANY($2) or line_id = $1
        ) as tmp
      )

      select way, latlng, node_id, array_to_json(edges) as edges
      from (
        select
          ST_AsHEXEWKB((line.point).geom) as way,
          ST_AsGeoJSON(ST_Force2D(ST_SwapOrdinates(ST_Transform((line.point).geom, 4326), 'xy')))::json->'coordinates' AS latlng,
          json_agg(distinct n.id) FILTER (WHERE n.id IS NOT NULL) as node_id,
          array_agg(distinct
            ARRAY[
              others.line_id,
              others.fraction,
              others.index
            ] -- ::test
          ) as edges
        from lines AS line
        join lines AS others on (others.point).geom = (line.point).geom
        left join nav_nodes n on ST_Intersects(n.way, (line.point).geom) and n.way = (line.point).geom
        where line.line_id = $1
        group by (line.point).geom, n.id --, line.fraction, line.index
        order by way
      ) as t
      where array_length(edges, 1) > 1
      OR edges[1][2] = 0
      OR edges[1][2] = 1
    )%"),
  m_countIntersectionPoints(
    m_dbConnection,
    R"%(
      select count(*)
      from (
        select point --, count(*)
        from (
          select (p0.point).geom AS point
          from (
            select ST_DumpPoints(l1.way2) AS point
            from planet_osm_route l1
            where ST_Intersects(l1.way, ST_SetSRID(ST_MakeBox2D(ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)), 3857)) 
          ) AS p0
          UNION ALL
          select ST_PointN(l1.way2, 1) AS point
          from planet_osm_route l1
          where ST_Intersects(l1.way, ST_SetSRID(ST_MakeBox2D(ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)), 3857)) 
          UNION ALL
          select ST_PointN(l1.way2, ST_NumPoints(l1.way2)) AS point
          from planet_osm_route l1
          where ST_Intersects(l1.way, ST_SetSRID(ST_MakeBox2D(ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)), 3857)) 
        ) AS p1
        where ST_Intersects(p1.point, ST_SetSRID(ST_MakeBox2D(ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)), 3857)) 
        group by point
        having count(*) > 1
      ) as total
    )%"
  ),
  m_queryExistingEdges (
    m_dbConnection,
    R"%(
      select
        nodes.id as node_id,
        ST_AsHEXEWKB(nodes.way) as way,
        ST_AsGeoJSON(ST_SwapOrdinates(ST_Force2D(ST_Transform(nodes.way, 4326)), 'xy'))::json->'coordinates' AS latlng,
        json_agg(
          json_build_object(
            'id', e2.id,
            'lineId', e2.line_id,
            'pointIndex', e2.point_index,
            'fraction', e2.fraction,
            'forwardCost', e2.forward_cost,
            'reverseCost', e2.reverse_cost,
            'forwardEdgeId', e2.forward_edge_id,
            'reverseEdgeId', e2.reverse_edge_id
          ) order by e2.line_id, e2.point_index, e2.id
        ) as edges
      from nav_nodes as nodes
      join nav_edges as e2 on e2.node_id = nodes.id
      -- join nav_edges as e3 on e3.node_id = nodes.id and e3.line_id = $1
      where nodes.id in (
	      select e.node_id
	      from nav_edges as e
	      where e.line_id = $1
      )
      group by nodes.id, way
      order by way
    )%"
  ),
  m_insertNode (
    m_dbConnection,
    R"%(
      insert into nav_nodes (way) values (decode($1, 'hex')) returning id
    )%"),
  m_deleteEdges (
    m_dbConnection,
    R"%(
      delete from nav_edges where node_id = $1
    )%"),
  m_intersectionQuery (
    m_dbConnection,
    R"%(
      select count(*) AS count
      from (
        select point
        from (
          select (p0.point).geom AS point
          from (
            select ST_DumpPoints(l1.way2) AS point
            from planet_osm_route l1
            where ST_Intersects(l1.way, ST_SetSRID(ST_MakeBox2D(ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)), 3857)) 
          ) AS p0
          UNION ALL
          select ST_PointN(l1.way2, 1) AS point
          from planet_osm_route l1
          where ST_Intersects(l1.way, ST_SetSRID(ST_MakeBox2D(ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)), 3857)) 
          UNION ALL
          select ST_PointN(l1.way2, ST_NumPoints(l1.way2)) AS point
          from planet_osm_route l1
          where ST_Intersects(l1.way, ST_SetSRID(ST_MakeBox2D(ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)), 3857)) 
        ) AS p1
        where ST_Intersects(p1.point, ST_SetSRID(ST_MakeBox2D(ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)), 3857)) 
        group by point
        having count(*) > 1
      ) as total
    )%"
  ),
  m_insertCount(
    m_dbConnection,
    R"%(
      INSERT INTO quad_intersection_counts (created_at, updated_at, lat, lng, intersection_count)
      VALUES (now(), now(), $1, $2, $3)
      ON CONFLICT (lat, lng)
      DO UPDATE SET updated_at = now(), intersection_count = $3
      RETURNING id
    )%"
  )
{
}

bool operator==(const Edge &a, const Edge &b)
{
  return a.pointIndex == b.pointIndex
    && a.lineId == b.lineId
    && a.fraction == b.fraction
    && a.reverseEdgeId == b.reverseEdgeId
    && a.forwardEdgeId == b.forwardEdgeId
    ;
}

bool operator<(const Edge &a, const Edge &b)
{
  return a.lineId < b.lineId || (a.lineId == b.lineId && a.pointIndex < b.pointIndex);
}


std::string serializeInsertEdge(
  EdgeUpdate edge)
{
  std::stringstream inserts;

  inserts /*<< std::setprecision(16) */
    << "(" << edge.edge->nodeId
    << ", "<< edge.edge->pointIndex
    << ", " << edge.edge->fraction
    << ", " << edge.edge->forwardCost
    << ", " << edge.edge->reverseCost
    << ", " << edge.edge->lineId
    << ", " << edge.edge->reverseEdgeId
    << ", " << edge.edge->forwardEdgeId
    << "),";

  return inserts.str();
}

std::tuple<std::vector<EdgeUpdate>, std::vector<EdgeUpdate>> addEdgeInserts(
  int nodeId,
  int lineId,
  std::vector<Edge>::iterator &iter2,
  std::vector<Edge> &newEdges
)
{
  std::vector<EdgeUpdate> lineEdges;
  std::vector<EdgeUpdate> otherEdges;

  // insert iter2 edges;
  for (; iter2 != newEdges.end(); ++iter2)
  {
    EdgeUpdate e(EdgeUpdate::Operation::insert, &(*iter2));
    e.edge->nodeId = nodeId;

    if (iter2->lineId == lineId)
    {
      lineEdges.push_back(e);
    }
    else
    {
      otherEdges.push_back(e);
      // inserts << serializeInsertEdge(e);
    }
  }

  return {lineEdges, otherEdges};
}

std::tuple<std::string, std::vector<EdgeUpdate>, std::vector<EdgeUpdate>> compareEdges(
  const std::vector<Edge> &existingEdges,
  int nodeId,
  int lineId,
  std::vector<Edge> &newEdges)
{
  auto iter1 = existingEdges.begin();
  auto iter2 = newEdges.begin();

  std::vector<EdgeUpdate> lineEdges;
  std::vector<EdgeUpdate> otherEdges;

  std::stringstream deletes;

  while (iter1 != existingEdges.end() && iter2 != newEdges.end())
  {
    if (iter1->lineId == iter2->lineId)
    {
      if (iter1->pointIndex < iter2->pointIndex)
      {
        // delete iter1 edge;
        deletes << iter1->id << ",";
        ++iter1;
      }
      else if (iter1->pointIndex > iter2->pointIndex)
      {
        EdgeUpdate e(EdgeUpdate::Operation::insert, &(*iter2));
        e.edge->nodeId = nodeId;

        if (iter2->lineId == lineId)
        {
          lineEdges.push_back(e);
        }
        else
        {
          otherEdges.push_back(e);
        }

        ++iter2;
      }
      else
      {
        // The point indexes are the same.

        EdgeUpdate e(EdgeUpdate::Operation::update, &(*iter2));
        e.edge->id = iter1->id;
        e.edge->nodeId = nodeId;
        e.edge->forwardCost = iter1->forwardCost;
        e.edge->reverseCost = iter1->reverseCost;
        e.edge->forwardEdgeId = iter1->forwardEdgeId;
        e.edge->reverseEdgeId = iter1->reverseEdgeId;

        if (iter1->lineId == lineId)
        {
          lineEdges.push_back(e);
        }
        else
        {
          otherEdges.push_back(e);
        }

        ++iter1;
        ++iter2;
      }
    }
    else if (iter1->lineId < iter2->lineId)
    {
      // delete iter1 edge
      deletes << iter1->id <<  ",";
      ++iter1;
    }
    else // iter1->lineId > iter2->lineId
    {
      EdgeUpdate e(EdgeUpdate::Operation::insert, &(*iter2));
      e.edge->nodeId = nodeId;

      if (iter2->lineId == lineId)
      {
        lineEdges.push_back(e);
      }
      else
      {
        otherEdges.push_back(e);
      }

      ++iter2;
    }
  }

  if (iter1 != existingEdges.end())
  {
    // delete iter1 edges;
    for (; iter1 != existingEdges.end(); ++iter1)
    {
      deletes << iter1->id <<  ",";
    }
  }
  else if (iter2 != newEdges.end())
  {
    std::vector<EdgeUpdate> newLineEdges;
    std::vector<EdgeUpdate> newOtherEdges;
    
    std::tie(newLineEdges, newOtherEdges) = addEdgeInserts(nodeId, lineId, iter2, newEdges);
    lineEdges.insert(lineEdges.end(), newLineEdges.begin(), newLineEdges.end());
    otherEdges.insert(otherEdges.end(), newOtherEdges.begin(), newOtherEdges.end());
  }

  return {deletes.str(), lineEdges, otherEdges};
}


GraphBuilder::EditLists GraphBuilder::makeEditLists(
  DBTransaction &transaction,
  const std::vector<ExistingNode> &existingNodes,
  std::vector<Node> &proposedNodes,
  int64_t lineId,
  Json::Value &status
)
{
  auto iter1 = existingNodes.begin();
  auto iter2 = proposedNodes.begin();

  EditLists edits;

  while (iter1 != existingNodes.end() && iter2 != proposedNodes.end())
  {
    if (iter1->m_way == iter2->m_way)
    {
      // The nodes are the same. Compare the edges.

      std::string deletes;
      std::vector<EdgeUpdate> newLineEdges;
      std::vector<EdgeUpdate> newOtherEdges;

      std::tie(deletes, newLineEdges, newOtherEdges) = compareEdges(
        iter1->m_edges,
        iter1->m_nodeId,
        lineId,
        iter2->m_edges);

      edits.deleteEdges += deletes;
      edits.lineEdges.insert(edits.lineEdges.end(), newLineEdges.begin(), newLineEdges.end());
      edits.otherEdges.insert(edits.otherEdges.end(), newOtherEdges.begin(), newOtherEdges.end());

      ++iter1;
      ++iter2;
    }
    else if (iter1->m_way < iter2->m_way)
    {
      // delete iter1 node
      edits.deleteNodes += std::to_string(iter1->m_nodeId) + ",";
      if (iter1->m_sameQuadrangle) {
        status["count"] = status["count"].asInt() - 1;
      }

      for (const auto &edge: iter1->m_edges)
      {
        edits.deleteEdges += std::to_string(edge.id) + ",";
      }

      ++iter1;
    }
    else // iter1->way > iter2->way
    {
      int nodeId = iter2->m_nodeId;

      // insert iter2 node
      if (nodeId == -1)
      {
        auto r  = transaction.exec1(m_insertNode, iter2->m_way);
        nodeId = r["id"].as<int64_t>();
        if (iter2->m_sameQuadrangle) {
          status["count"] = status["count"].asInt() + 1;
        }
      }

      iter2->m_nodeId = nodeId;

      auto edgeIter = iter2->m_edges.begin();

      std::vector<EdgeUpdate> newLineEdges;
      std::vector<EdgeUpdate> newOtherEdges;
      
      std::tie(newLineEdges, newOtherEdges) = addEdgeInserts(nodeId, lineId, edgeIter, iter2->m_edges);
      edits.lineEdges.insert(edits.lineEdges.end(), newLineEdges.begin(), newLineEdges.end());
      edits.otherEdges.insert(edits.otherEdges.end(), newOtherEdges.begin(), newOtherEdges.end());

      ++iter2;
    }
  }

  if (iter1 != existingNodes.end())
  {
    // delete iter1 nodes
    for (; iter1 != existingNodes.end(); ++iter1)
    {
      edits.deleteNodes += std::to_string(iter1->m_nodeId) + ",";
      if (iter1->m_sameQuadrangle) {
        status["count"] = status["count"].asInt() - 1;
      }

      for (const auto &edge: iter1->m_edges)
      {
        edits.deleteEdges += std::to_string(edge.id) + ",";
      }
    }
  }
  else if (iter2 != proposedNodes.end())
  {
    // insert iter2 nodes
    for (; iter2 != proposedNodes.end(); ++iter2)
    {
      int nodeId = iter2->m_nodeId;

      // insert iter2 node
      if (nodeId == -1)
      {
        auto r  = transaction.exec1(m_insertNode, iter2->m_way);
        nodeId = r["id"].as<int64_t>();
        if (iter2->m_sameQuadrangle) {
          status["count"] = status["count"].asInt() + 1;
        }
      }

      auto edgeIter = iter2->m_edges.begin();

      std::vector<EdgeUpdate> newLineEdges;
      std::vector<EdgeUpdate> newOtherEdges;
    
      std::tie(newLineEdges, newOtherEdges) = addEdgeInserts(nodeId, lineId, edgeIter, iter2->m_edges);
      edits.lineEdges.insert(edits.lineEdges.end(), newLineEdges.begin(), newLineEdges.end());
      edits.otherEdges.insert(edits.otherEdges.end(), newOtherEdges.begin(), newOtherEdges.end());

      iter2->m_nodeId = nodeId;
    }
  }

  return edits;
}


void insertLineEdges(
  DBTransaction &transaction,
  std::vector<EdgeUpdate> &lineEdges
)
{
  std::string insertEdges;

  for (size_t i = 0; i < lineEdges.size(); i++)
  {
    if (lineEdges[i].operation == EdgeUpdate::Operation::insert) {
      insertEdges += serializeInsertEdge(lineEdges[i]); // .nodeId, -1, -1, *lineEdges[i].edge1);
    }
  }

  if (!insertEdges.empty())
  {
    auto result = transaction.exec(
      R"%(
        insert into nav_edges (node_id, point_index, fraction, forward_cost, reverse_cost, line_id, reverse_edge_id, forward_edge_id)
        values
      )%"
    + insertEdges.substr(0, insertEdges.size() - 1)
    + " returning id");

    for (size_t i = 0, j = 0; i < lineEdges.size(); i++)
    {
      if (lineEdges[i].operation == EdgeUpdate::Operation::insert)
      {
        lineEdges[i].edge->id = result[j++]["id"].as<int>();
      }
    }
  }
}

std::tuple<double, double> GraphBuilder::getEdgeCosts(
  DBTransaction &transaction,
  int lineId,
  double startFraction,
  double endFraction
)
{
  PreparedStatement queryEdges(
    transaction,
    R"%(
      select
        ST_AsGeoJSON(ST_SwapOrdinates(ST_Transform(ST_LineSubString(way2, $2, $3), 4326), 'xy'))::json->'coordinates' as line
      from planet_osm_route as route
      where route.line_id = $1
    )%"
  );

  auto row = transaction.exec1(queryEdges, lineId, startFraction, endFraction);

  Json::Reader reader;

  Json::Value line;
  reader.parse(row["line"].as<std::string>(), line);

  return computeCosts(line);
}

void GraphBuilder::setEdgeRelationsAndCosts(
  DBTransaction &transaction,
  int lineId,
  std::vector<EdgeUpdate> &lineEdges
)
{
  // Sort the edges by point index.
  std::sort(lineEdges.begin(), lineEdges.end(),
    [](const EdgeUpdate &a, const EdgeUpdate &b)
    {
      return a.edge->pointIndex < b.edge->pointIndex;
    });

  // Now that we have the ordering of the nodes, we can
  // determine backward and forward edge IDs for the edges
  // and insert, update or delete them.
  for (size_t i = 1; i < lineEdges.size(); i++)
  {
    lineEdges[i - 1].edge->forwardEdgeId = lineEdges[i].edge->id;
    lineEdges[i].edge->reverseEdgeId = lineEdges[i - 1].edge->id;

    double forwardCost;
    double reverseCost;

    std::tie(forwardCost, reverseCost) = getEdgeCosts(
      transaction, lineId,
      lineEdges[i - 1].edge->fraction,
      lineEdges[i].edge->fraction);

    lineEdges[i - 1].edge->forwardCost = forwardCost;
    lineEdges[i].edge->reverseCost = reverseCost;
  }
}

void updateEdges(
  DBTransaction &transaction,
  std::vector<EdgeUpdate> &lineEdges
)
{
  std::stringstream edgeUpdates;

  for (const auto &edge: lineEdges)
  {
    edgeUpdates << "("
      << edge.edge->id << ", "
      << edge.edge->fraction << ", "
      << edge.edge->forwardEdgeId << ", "
      << edge.edge->reverseEdgeId << ", "
      << edge.edge->forwardCost << ", "
      << edge.edge->reverseCost << "),";
  }

  if (!edgeUpdates.str().empty())
  {
    auto updates = edgeUpdates.str();
    updates.pop_back(); // Pops off the trailing ","

    transaction.exec(
      R"%(
        update nav_edges
        set
          fraction = t.fraction,
          forward_edge_id = t.forward_edge_id,
          reverse_edge_id = t.reverse_edge_id,
          forward_cost = t.forward_cost,
          reverse_cost = t.reverse_cost
        from (values )%"
      + updates
      + R"%(
        ) as t(edge_id, fraction, forward_edge_id, reverse_edge_id, forward_cost, reverse_cost)
        where nav_edges.id = t.edge_id
      )%"
    );
  }
}

void deleteEdgesAndNodes(
  DBTransaction &transaction,
  std::string deleteEdges,
  std::string deleteNodes
)
{
  if (!deleteEdges.empty())
  {
    transaction.exec("delete from nav_edges where id in (" + deleteEdges.substr(0, deleteEdges.size() - 1) + ")");
  }

  if (!deleteNodes.empty())
  {
    transaction.exec("delete from nav_nodes where id in (" + deleteNodes.substr(0, deleteNodes.size() - 1) + ")");
  }
}

void GraphBuilder::modifyNodes(
  DBTransaction &transaction,
  const std::vector<ExistingNode> &existingNodes,
  std::vector<Node> &proposedNodes,
  int64_t lineId,
  Json::Value &status)
{
  auto edits = makeEditLists(transaction, existingNodes, proposedNodes, lineId, status);

  // Insert the new edges so that we can get the edge ids.
  insertLineEdges(transaction, edits.lineEdges);
  insertLineEdges(transaction, edits.otherEdges);

  setEdgeRelationsAndCosts(transaction, lineId, edits.lineEdges);

  deleteEdgesAndNodes(transaction, edits.deleteEdges, edits.deleteNodes);

  updateEdges(transaction, edits.lineEdges);
  updateEdges(transaction, edits.otherEdges);
}

Profiler eachIteration("eachIteration");
Profiler queryIntersections("queryIntersections");
Profiler processNew("processNew");
Profiler queryExisting("queryExisting");
Profiler processExisting("processExisting");
Profiler createEditList("createEditList");
Profiler applyEdits("applyEdits");

std::tuple<Node, std::vector<Json::Value>> GraphBuilder::makeNodeFromRow (
  const pqxx::row &row,
  const LatLngBounds &bounds
)
{
	Json::Reader reader;
  Json::Value ids;
  reader.parse(row["node_id"].as<std::string>(), ids);

  std::set<Json::Value> tempSet(ids.begin(), ids.end());
  std::vector<Json::Value> nodeIds(tempSet.begin(), tempSet.end());
  std::sort(nodeIds.begin(), nodeIds.end());

  // Use the first node on this intersection, if any
  int nodeId = -1;
  if (nodeIds.size() > 0) {
    nodeId = nodeIds.front().asInt();
    nodeIds.erase(nodeIds.begin());
  }

  Json::Value latlng;
  reader.parse(row["latlng"].as<std::string>(), latlng);
  bool sameQuadrangle = bounds.contains(latlng);

  Node node {nodeId, row["way"].as<std::string>(), sameQuadrangle};

  // auto pointIndex = row["pointIndex"].as<int>();
  // node.m_currentEdges.emplace_back(-1, lineId1, pointIndex, row["fraction"].as<double>());

  // Add the edges to the node.
  Json::Value edges;
  reader.parse(row["edges"].as<std::string>(), edges);

  for (const auto &edge: edges)
  {
    auto lineId = edge[0].asInt();
    auto fraction = edge[1].asDouble();
    auto pointIndex = edge[2].asInt();

    // if (otherLineId != lineId1 || otherPointIndex != pointIndex)
    {
      node.m_edges.emplace_back(
        lineId,
        pointIndex,
        fraction
      );
    }
  }

  std::set<Edge> set(node.m_edges.begin(), node.m_edges.end());
  node.m_edges.assign(set.begin(), set.end());
  std::sort(node.m_edges.begin(), node.m_edges.end());

  return {node, nodeIds};
}


void deleteNodes(
  DBTransaction &transaction,
  std::vector<Json::Value> nodeIds,
  bool sameQuadrangle,
  Json::Value &status
)
{
  // Delete any duplicate nodes on this intersection
  if (nodeIds.size() > 0)
  {
    std::string deleteNodes;
    for (size_t i = 0; i < nodeIds.size(); i++)
    {
      deleteNodes += nodeIds[i].asString() + ",";

      if (sameQuadrangle) {
        status["count"] = status["count"].asInt() - 1;
      }
    }

    transaction.exec("delete from nav_edges where node_id in (" + deleteNodes.substr(0, deleteNodes.size() - 1) + ")");
    transaction.exec("delete from nav_nodes where id in (" + deleteNodes.substr(0, deleteNodes.size() - 1) + ")");
  }
}


std::vector<Node> GraphBuilder::getProposedNodes(
  DBTransaction &transaction,
  int64_t lineId,
  const std::string &others,
  const LatLngBounds &bounds,
  Json::Value &status
)
{
  std::vector<Node> newNodes;

  queryIntersections.start();
  auto rows = transaction.exec(m_queryIntersectionPoints, lineId, others);
  queryIntersections.stop();

  for (const auto &row: rows)
  {
    Node node;
    std::vector<Json::Value> nodeIds;

    std::tie(node, nodeIds) = makeNodeFromRow(row, bounds);

    newNodes.push_back(node);

    deleteNodes(transaction, nodeIds, node.m_sameQuadrangle, status);
  }

  return newNodes;
}


std::vector<ExistingNode> GraphBuilder::getExistingNodes (
  DBTransaction &transaction,
  int64_t lineId,
  const LatLngBounds &bounds
)
{
	Json::Reader reader;
  std::vector<ExistingNode> existingNodes;

  queryExisting.start();
  auto rows = transaction.exec(m_queryExistingEdges, lineId);
  queryExisting.stop();

  for (const auto &row: rows)
  {
    Json::Value latlng;
    std::cout << row["latlng"].as<std::string>() << std::endl;
    reader.parse(row["latlng"].as<std::string>(), latlng);
    bool sameQuadrangle = bounds.contains(latlng);

    ExistingNode node {row["node_id"].as<int>(), row["way"].as<std::string>(), sameQuadrangle};

    Json::Value edges;
    reader.parse(row["edges"].as<std::string>(), edges);

    for (const auto &edge: edges)
    {
      double fraction = -1;

      if (!edge["fraction"].isNull())
      {
        fraction = edge["fraction"].asDouble();
      }

      Edge e(edge["lineId"].asInt(), edge["pointIndex"].asInt(), fraction);
      e.id = edge["id"].asInt();
      e.nodeId = node.m_nodeId;
      e.forwardCost = edge["forwardCost"].asDouble();
      e.reverseCost = edge["reverseCost"].asDouble();
      e.forwardEdgeId = edge["forwardEdgeId"].asInt();
      e.reverseEdgeId = edge["reverseEdgeId"].asInt();

      node.m_edges.push_back(e);
    }

    existingNodes.push_back(node);
  }

  return existingNodes;
}


void GraphBuilder::updateIntersections (
  DBTransaction &transaction,
  const LatLngBounds &bounds,
  pqxx::result &intersections,
  int nodeCount,
  int max)
{
	Json::Reader reader;
  Json::Value status = Json::objectValue;

  status["status"] = StatusRoutesUpdated;
  status["lat"] = bounds.m_southWest.m_lat;
  status["lng"] = bounds.m_southWest.m_lng;
  status["count"] = nodeCount;
  status["max"] = max;
  statusUpdate(status);

  auto previousTime = std::chrono::steady_clock::now();

  std::cerr << "Number of routes to process: " << intersections.size() << std::endl;

  int routesProcessed {0};

	// Iterate through the list of intersections
	for (const auto &intersection: intersections)
	{
    eachIteration.start();
		auto lineId = intersection["line_id"].as<int64_t>();
    auto others = intersection["other_lines"].as<std::string>();

    processNew.start();
    auto newNodes = getProposedNodes(transaction, lineId, others, bounds, status);
    processNew.stop();
    
    processExisting.start();
    auto existingNodes = getExistingNodes(transaction, lineId, bounds);
    processExisting.stop();

    createEditList.start();
    modifyNodes(transaction, existingNodes, newNodes, lineId, status);
    createEditList.stop();

    eachIteration.stop();
    
    routesProcessed++;

    auto now = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(now - previousTime);

    if (elapsedTime.count() >= 1)
    {
      statusUpdate(status);
      previousTime = now;
    }
	}

  statusUpdate(status);
}


int GraphBuilder::updateIntersectionCount(
  DBTransaction &transaction,
  int lat,
  int lng)
{
  auto count = transaction.exec1(m_intersectionQuery, lng, lat);

  auto id = transaction.exec1(m_insertCount, lat, lng, count["count"].as<int>());

  Json::Value status = Json::objectValue;
  status["status"] = StatusUpdateIntersectionCount;
  status["lat"] = lat;
  status["lng"] = lng;
  status["count"] = count["count"].as<int>();

  statusUpdate(status);

  return count["count"].as<int>();
}

std::tuple<int, int> GraphBuilder::getNodeCounts(DBTransaction &transaction, int lat, int lng)
{
  PreparedStatement nodeCounts(
    m_dbConnection,
    R"%(
      select COALESCE(t1.count, 0) as count, intersection_count as max
      from quad_intersection_counts as q
      left join (
            select floor(ST_X(ST_Transform(n.way, 4326))) as lng, floor(ST_Y(ST_Transform(n.way, 4326))) as lat, count(*)
            from nav_nodes AS n
            where ST_Intersects(
              n.way,
              ST_SetSRID(ST_MakeBox2D(
                ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857),
                ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)
              ), 3857)
            ) 
            group by lng, lat
      ) as t1 on q.lat = t1.lat and q.lng = t1.lng
      where q.lng = $1 and q.lat = $2
    )%"
  );

  auto result = transaction.exec1(nodeCounts, lng, lat);

  int count = result["count"].as<int>();
  int max {0};

  if (result["max"].is_null())
  {
    max = updateIntersectionCount(transaction, lat, lng);
  }
  else
  {
    max = result["max"].as<int>();
  }

  return {count, max};
}

void GraphBuilder::buildGraphInArea(
  const LatLngBounds &bounds
) {
  auto transaction = m_dbConnection->newTransaction();

  buildGraphInArea(transaction, bounds, 0, 0);

  transaction.commit();
}

void GraphBuilder::buildGraphInArea(
  DBTransaction &transaction,
  const LatLngBounds &bounds,
  int nodeCount,
  int max
) {
	auto intersections = transaction.exec(
    m_queryIntersections,
    bounds.m_southWest.m_lng, bounds.m_southWest.m_lat,
    bounds.m_northEast.m_lng, bounds.m_northEast.m_lat);
  
	updateIntersections (
    transaction,
    bounds,
    intersections,
    nodeCount,
    max);
}

void GraphBuilder::buildGraph (
	double lat,
	double lng)
{
  Json::Value status = Json::objectValue;

  status["status"] = StatusUpdatingRoutes;
  status["lat"] = lat;
  status["lng"] = lng;

  statusUpdate(status);

  auto transaction = m_dbConnection->newTransaction();

  int nodeCount, max;
  std::tie(nodeCount, max) = getNodeCounts(transaction, lat, lng);

  LatLngBounds bounds(lat, lng, lat + 1, lng + 1);
  buildGraphInArea(transaction, bounds, nodeCount, max);

  std::tie(status["count"], status["max"]) = getNodeCounts(transaction, lat, lng);
  status["status"] = StatusRoutesUpdated;
  statusUpdate(status);

  transaction.commit();
}

} // namespace gb
