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
      where l1.way && ST_SetSRID(ST_MakeBox2D(ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)), 3857)
      AND ST_Intersects(l1.way, ST_SetSRID(ST_MakeBox2D(ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857), ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)), 3857))
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
      select
        ST_AsHEXEWKB((line.point).geom) as way,
        ST_AsGeoJSON(ST_Force2D(ST_Transform((line.point).geom, 4326)))::json->'coordinates' AS latlng,
        json_agg(distinct COALESCE(n.id, -1)) as node_id,
        min(line.fraction) AS fraction,
        min(line.index) as pointIndex,
        array_to_json(array_agg(distinct
          row(
            others.line_id,
            others.fraction,
            others.index
          ) -- ::test
        ) FILTER (WHERE others.line_id != line.line_id OR others.fraction > line.fraction)) as others
      from lines AS line
      join lines AS others on (others.point).geom = (line.point).geom
    	and (others.line_id != line.line_id OR (others.point).path[1] != (line.point).path[1])
      left join nav_nodes n on ST_Intersects(n.way, (line.point).geom) and n.way = (line.point).geom
      where line.line_id = $1
      group by (line.point).geom, n.id
      order by way
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
        ST_AsGeoJSON(ST_Force2D(ST_Transform(nodes.way, 4326)))::json->'coordinates' AS latlng,
        json_agg(
          json_build_object(
            'id', e2.id,
            'lineId', e2.line_id,
            'pointIndex', e2.point_index,
            'fraction', e2.fraction
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
    && a.forwardEdgeId == b.forwardEdgeId;
}

bool operator<(const Edge &a, const Edge &b)
{
  return a.lineId < b.lineId || (a.lineId == b.lineId && a.pointIndex < b.pointIndex);
}


std::string insertEdge(
  int nodeId,
  int reverseEdgeId,
  int forwardEdgeId,
  const Edge &edge)
{
  std::stringstream inserts;

  inserts /*<< std::setprecision(16) */<< "(" << nodeId << ", " << edge.pointIndex << ", " << edge.fraction << ", " << 0 << ", " << 0 << ", " << edge.lineId;
  inserts << ", " << reverseEdgeId << ", " << forwardEdgeId << "),";

  return inserts.str();
}

enum class Operation
{
  update,
  insert
};

/// using UpdateRec = std::tuple<int, Operation, int, const Edge *, const Edge *>;

struct UpdateRec
{
  UpdateRec(int id, int pi, Operation o, int ni, const Edge *e1, const Edge *e2)
  :
    edgeId (id),
    pointIndex(pi),
    operation(o),
    nodeId(ni),
    edge1(e1),
    edge2(e2)
  {}

  int edgeId {-1};
  int pointIndex;
  Operation operation;
  int nodeId;
  const Edge *edge1;
  const Edge *edge2;
};

std::tuple<std::string, std::string, std::vector<UpdateRec>> checkEdges(
  const std::vector<Edge> &existingEdges,
  int nodeId,
  int lineId,
  const std::vector<Edge> &newEdges)
{
  auto iter1 = existingEdges.begin();
  auto iter2 = newEdges.begin();

  std::vector<UpdateRec> lineEdges;

  std::stringstream inserts;
  std::stringstream deletes;

  // inserts << std::setprecision(16);

  while (iter1 != existingEdges.end() && iter2 != newEdges.end())
  {
    if (iter1->lineId == iter2->lineId)
    {
      if (iter1->pointIndex < iter2->pointIndex)
      {
        // delete iter1 edge;
        deletes << iter1->edgeId << ",";
        ++iter1;
      }
      else if (iter1->pointIndex > iter2->pointIndex)
      {
        if (iter2->lineId == lineId)
        {
          lineEdges.emplace_back(-1, iter2->pointIndex, Operation::insert, nodeId, &(*iter2), nullptr);
        }
        else
        {
          inserts << insertEdge(nodeId, -1, -1, *iter2);
        }
        ++iter2;
      }
      else
      {
        if (iter1->lineId == lineId)
        {
          lineEdges.emplace_back(iter1->edgeId, iter1->pointIndex, Operation::update, nodeId, &(*iter1), &(*iter2));
        }
        // else if (iter1->fraction != iter2->fraction)
        // {
        //   updates << "(" << iter1->edgeId << ", " << iter2->fraction << ", -1, -1),";
        // }

        ++iter1;
        ++iter2;
      }
    }
    else if (iter1->lineId < iter2->lineId)
    {
      // delete iter1 edge
      deletes << iter1->edgeId <<  ",";
      ++iter1;
    }
    else // iter1->lineId > iter2->lineId
    {
      if (iter2->lineId == lineId)
      {
        lineEdges.emplace_back(-1, iter2->pointIndex, Operation::insert, nodeId, &(*iter2), nullptr);
      }
      else
      {
        // insert iter2 edge
        inserts << insertEdge(nodeId, -1, -1, *iter2);
      }

      ++iter2;
    }
  }

  if (iter1 != existingEdges.end())
  {
    // delete iter1 edges;
    for (; iter1 != existingEdges.end(); ++iter1)
    {
      deletes << iter1->edgeId <<  ",";
    }
  }
  else if (iter2 != newEdges.end())
  {
    // insert iter2 edges;
    for (; iter2 != newEdges.end(); ++iter2)
    {
      if (iter2->lineId == lineId)
      {
        lineEdges.emplace_back(-1, iter2->pointIndex, Operation::insert, nodeId, &(*iter2), nullptr);
      }
      else
      {
        inserts << insertEdge(nodeId, -1, -1, *iter2);
      }
    }
  }

  return {inserts.str(), deletes.str(), lineEdges};
}

void GraphBuilder::modifyNodes(
  DBTransaction &transaction,
  const std::vector<Node> &existingNodes,
  std::vector<Node> &proposedNodes,
  int64_t lineId,
  Json::Value &status)
{
  auto iter1 = existingNodes.begin();
  auto iter2 = proposedNodes.begin();

  std::vector<UpdateRec> lineEdges;
  
  std::string deleteNodes;
  std::stringstream insertEdges;
  std::string deleteEdges;
  std::stringstream updateEdges;

  while (iter1 != existingNodes.end() && iter2 != proposedNodes.end())
  {
    if (iter1->m_way == iter2->m_way)
    {
//      if (iter1->m_currentEdges != iter2->m_currentEdges)
      {
        std::string inserts;
        std::string deletes;
        std::vector<UpdateRec> newLineEdges;

        std::tie(inserts, deletes, newLineEdges) = checkEdges(
          iter1->m_currentEdges,
          iter1->m_nodeId,
          lineId,
          iter2->m_currentEdges);

        insertEdges << inserts;
        deleteEdges += deletes;
        lineEdges.insert(lineEdges.end(), newLineEdges.begin(), newLineEdges.end());
      }

      ++iter1;
      ++iter2;
    }
    else if (iter1->m_way < iter2->m_way)
    {
      // delete iter1 node
      deleteNodes += std::to_string(iter1->m_nodeId) + ",";
      if (iter1->m_sameQuadrangle) {
        status["count"] = status["count"].asInt() - 1;
      }

      for (const auto &edge: iter1->m_currentEdges)
      {
        deleteEdges += std::to_string(edge.edgeId) + ",";
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

      {
        std::string inserts;
        std::string deletes;
        std::vector<UpdateRec> newLineEdges;

        std::tie(inserts, deletes, newLineEdges) = checkEdges(
          {},
          nodeId,
          lineId,
          iter2->m_currentEdges);

        insertEdges << inserts;
        deleteEdges += deletes;
        lineEdges.insert(lineEdges.end(), newLineEdges.begin(), newLineEdges.end());
      }

      ++iter2;
    }
  }

  if (iter1 != existingNodes.end())
  {
    // delete iter1 nodes
    for (; iter1 != existingNodes.end(); ++iter1)
    {
      deleteNodes += std::to_string(iter1->m_nodeId) + ",";
      if (iter1->m_sameQuadrangle) {
        status["count"] = status["count"].asInt() - 1;
      }

      for (const auto &edge: iter1->m_currentEdges)
      {
        deleteEdges += std::to_string(edge.edgeId) + ",";
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

      {
        std::string inserts;
        std::string deletes;
        std::vector<UpdateRec> newLineEdges;

        std::tie(inserts, deletes, newLineEdges) = checkEdges(
          {},
          nodeId,
          lineId,
          iter2->m_currentEdges);

        insertEdges << inserts;
        deleteEdges += deletes;
        lineEdges.insert(lineEdges.end(), newLineEdges.begin(), newLineEdges.end());
      }

      iter2->m_nodeId = nodeId;
    }
  }

  std::sort(lineEdges.begin(), lineEdges.end(),
    [](const UpdateRec &a, const UpdateRec &b)
    {
      return a.pointIndex < b.pointIndex;
    });

  // Insert the new edges so that we can get the edge ids.
  {
    std::string insertEdges;

    for (size_t i = 0; i < lineEdges.size(); i++)
    {
      if (lineEdges[i].operation == Operation::insert) {
        insertEdges += insertEdge(lineEdges[i].nodeId, -1, -1, *lineEdges[i].edge1);
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
        if (lineEdges[i].operation == Operation::insert)
        {
          lineEdges[i].edgeId = result[j++]["id"].as<int>();
        }
      }
    }
  }

  double prevFraction {-1};

  // Now that we have the ordering of the nodes, we can
  // determine backward and forward node IDs for the edges
  // and insert, update or delete them.
  for (size_t i = 0; i < lineEdges.size(); i++)
  {
    int reverseEdgeId {-1};
    int forwardEdgeId {-1};

    if (i > 0)
    {
      reverseEdgeId = lineEdges[i - 1].edgeId;
    }

    if (i + 1 < lineEdges.size())
    {
      forwardEdgeId = lineEdges[i + 1].edgeId;
    }

    switch (lineEdges[i].operation)
    {
      case Operation::update:
        if (lineEdges[i].edge1->fraction != lineEdges[i].edge2->fraction
          || lineEdges[i].edge1->forwardEdgeId != forwardEdgeId
          || lineEdges[i].edge1->reverseEdgeId != reverseEdgeId)
        {
          updateEdges << "("
            << lineEdges[i].edgeId << ", "
            << lineEdges[i].edge2->fraction << ", "
            << reverseEdgeId << ", "
            << forwardEdgeId << "),";
        }

        if (prevFraction >= 0 && prevFraction > lineEdges[i].edge2->fraction) {
          std::cerr << "fraction ordering issue?" << std::endl;              
        }

        prevFraction = lineEdges[i].edge2->fraction;

        break;

      case Operation::insert:
        // The record was inserted in a previous step to get the assigned edge id
        // so now it is just an update to set the reverse and forward edge ids.
        updateEdges << "("
          << lineEdges[i].edgeId << ", "
          << lineEdges[i].edge1->fraction << ", "
          << reverseEdgeId << ", "
          << forwardEdgeId << "),";

        if (prevFraction >= 0 && prevFraction > lineEdges[i].edge1->fraction) {
          std::cerr << "fraction ordering issue?" << std::endl;              
        }

        prevFraction = lineEdges[i].edge1->fraction;
        break;
    }
  }

  if (!insertEdges.str().empty())
  {
    auto inserts = insertEdges.str();
    transaction.exec(
      R"%(
        insert into nav_edges (node_id, point_index, fraction, forward_cost, reverse_cost, line_id, reverse_edge_id, forward_edge_id)
        values
      )%"
    + inserts.substr(0, inserts.size() - 1));
  }

  if (!deleteEdges.empty())
  {
    transaction.exec("delete from nav_edges where id in (" + deleteEdges.substr(0, deleteEdges.size() - 1) + ")");
  }

  if (!deleteNodes.empty())
  {
    transaction.exec("delete from nav_nodes where id in (" + deleteNodes.substr(0, deleteNodes.size() - 1) + ")");
  }

  if (!updateEdges.str().empty())
  {
    auto updates = updateEdges.str();
    updates.pop_back();
  
    transaction.exec(
      R"%(
        update nav_edges
        set fraction = t.fraction,
          reverse_edge_id = t.reverse_edge_id,
          forward_edge_id = t.forward_edge_id
        from (values )%"
      + updates
      + ") as t(edge_id, fraction, reverse_edge_id, forward_edge_id) where t.edge_id = nav_edges.id");
  }
}

Profiler eachIteration("eachIteration");
Profiler queryIntersections("queryIntersections");
Profiler processNew("processNew");
Profiler queryExisting("queryExisting");
Profiler processExisting("processExisting");
Profiler createEditList("createEditList");
Profiler applyEdits("applyEdits");

void GraphBuilder::getIntersections (
  DBTransaction &transaction,
  double lat,
  double lng,
  pqxx::result &intersections,
  int nodeCount,
  int max)
{
	Json::Reader reader;
  Json::Value status = Json::objectValue;

  status["status"] = StatusRoutesUpdated;
  status["lat"] = lat;
  status["lng"] = lng;
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
		auto lineId1 = intersection["line_id"].as<int64_t>();

    std::vector<Node> newNodes;
    std::vector<Node> existingNodes;

    queryIntersections.start();
    auto others = intersection["other_lines"].as<std::string>();
  	auto result = transaction.exec(m_queryIntersectionPoints, lineId1, others);
    queryIntersections.stop();

    processNew.start();
    for (const auto &row: result)
    {
      Json::Value ids;
      reader.parse(row["node_id"].as<std::string>(), ids);

      std::set<Json::Value> tempSet(ids.begin(), ids.end());
      std::vector<Json::Value> nodeIds(tempSet.begin(), tempSet.end());
      std::sort(nodeIds.begin(), nodeIds.end());

      // Get the first node on this intersection, ignoring any -1 values
      int nodeId = -1;
      for (const auto &id: nodeIds)
      {
        if (id.asInt() != -1)
        {
          nodeId = id.asInt();
          break;
        }
      }

      Json::Value latlng;
      reader.parse(row["latlng"].as<std::string>(), latlng);
      double nodeLat = std::floor(latlng[1].asDouble());
      double nodeLng = std::floor(latlng[0].asDouble());
      bool sameQuadrangle = (lat = nodeLat && lng == nodeLng);

      Node node {nodeId, row["way"].as<std::string>(), sameQuadrangle};

      // Json::Value t1Edges;
      // reader.parse(row["others"].as<std::string>(), t1Edges);

      // for (const auto &edge: t1Edges)
      // {
      //   node.m_currentEdges.emplace_back(-1, edge["lineId"].asInt(), edge["fraction"].asDouble());
      // }

      node.m_currentEdges.emplace_back(-1, lineId1, row["pointIndex"].as<int>(), row["fraction"].as<double>());

      Json::Value edges;
      reader.parse(row["others"].as<std::string>(), edges);

      for (const auto &edge: edges)
      {
        if (!edge.isNull())
        {
          node.m_currentEdges.emplace_back(-1, edge["f1"].asInt(), edge["f3"].asInt(), edge["f2"].asDouble());
        }
      }

      // Delete any duplicate nodes on this intersection
      if (nodeIds.size() > 1)
      {
        std::string deleteNodes;
        for (size_t i = 1; i < nodeIds.size(); i++)
        {
          deleteNodes += nodeIds[i].asString() + ",";

          if (sameQuadrangle) {
            status["count"] = status["count"].asInt() - 1;
          }
        }

        transaction.exec("delete from nav_edges where node_id in (" + deleteNodes.substr(0, deleteNodes.size() - 1) + ")");
        transaction.exec("delete from nav_nodes where id in (" + deleteNodes.substr(0, deleteNodes.size() - 1) + ")");
      }

      std::set<Edge> set(node.m_currentEdges.begin(), node.m_currentEdges.end());
      node.m_currentEdges.assign(set.begin(), set.end());
      std::sort(node.m_currentEdges.begin(), node.m_currentEdges.end());

      newNodes.push_back(node);
    }
    processNew.stop();
    
    queryExisting.start();
  	result = transaction.exec(m_queryExistingEdges, lineId1);
    queryExisting.stop();

    processExisting.start();
    for (const auto &row: result)
    {
      Json::Value latlng;
      reader.parse(row["latlng"].as<std::string>(), latlng);
      double nodeLat = std::floor(latlng[1].asDouble());
      double nodeLng = std::floor(latlng[0].asDouble());
      bool sameQuadrangle = (lat = nodeLat && lng == nodeLng);

      Node node {row["node_id"].as<int>(), row["way"].as<std::string>(), sameQuadrangle};

      Json::Value edges;
      reader.parse(row["edges"].as<std::string>(), edges);

      for (const auto &edge: edges)
      {
        double fraction = -1;

        if (!edge["fraction"].isNull())
        {
          fraction = edge["fraction"].asDouble();
        }

        node.m_currentEdges.emplace_back(edge["id"].asInt(), edge["lineId"].asInt(), edge["pointIndex"].asInt(), fraction);
      }

      existingNodes.push_back(node);
    }
    processExisting.stop();

    createEditList.start();
    modifyNodes(transaction, existingNodes, newNodes, lineId1, status);
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

	auto intersections = transaction.exec(m_queryIntersections, lng, lat);
	getIntersections (transaction, lat, lng, intersections, nodeCount, max);

  std::tie(status["count"], status["max"]) = getNodeCounts(transaction, lat, lng);
  status["status"] = StatusRoutesUpdated;
  statusUpdate(status);

  transaction.commit();
}

} // namespace gb
