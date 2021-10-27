/*
 * Map.cpp
 *
 *  Created on: Nov 3, 2019
 *      Author: richard
 */

#include <pqxx/pqxx>
#include "Map.h"
#include "Elevation.h"
#include "./Database/DBConnection.h"
#include "Cost.h"
#include <jsoncpp/json/json.h>
#include <iostream>


void stringReplace(const std::string &oldValue, const std::string &newValue,
		std::string &str)
{
	size_t index = 0;
	while (index != std::string::npos)
	{
		// Locate the substring to replace.
		index = str.find(oldValue, index);
		if (index != std::string::npos)
		{
			// Make the replacement.
			str.replace(index, oldValue.length(), newValue);

			// Advance index forward
			index += newValue.length();
		}
	}
}

TrailInfo getTrailFromPoint(const Point &point)
{
	try
	{
    auto dbConnection = std::make_shared<DBConnection>();
		std::stringstream p;

		p << std::setprecision(15) << point.m_lng << " " << point.m_lat << std::defaultfloat;

		std::string sql = R"%(
      SELECT 
        ST_Distance(L2.way, L2.original_point) AS distance, 
        ST_AsGeoJSON(ST_Transform(L2.point, 4326))::json->'coordinates' AS point, 
        L2.fraction as fraction,
        L2.line_id,
        COALESCE(start_edge.id, -1) AS start_edge_id,
        COALESCE(end_edge.id, -1) AS end_edge_id,
        start_edge.fraction as start_fraction,
        COALESCE(end_edge.fraction, start_edge.fraction) as end_fraction, 
        start_edge.node_id as start_node,
        COALESCE(end_edge.node_id, -1) as end_node, 
        start_edge.forward_cost as forward_cost,
        COALESCE(end_edge.reverse_cost, -1) as reverse_cost
      FROM (
        SELECT
          way,
          line_id,
          ST_Transform('SRID=4326;POINT(:point:)'::geometry, 3857) as original_point,
          ST_ClosestPoint(way, ST_Transform('SRID=4326;POINT(:point:)'::geometry, 3857)) as point,
          ST_LineLocatePoint(way, ST_ClosestPoint(way, ST_Transform('SRID=4326;POINT(:point:)'::geometry, 3857))) as fraction
        FROM planet_osm_route AS R
        ORDER BY way <-> ST_Transform('SRID=4326;POINT(:point:)'::geometry, 3857)
        LIMIT 10
      ) L2 
      LEFT JOIN nav_edges AS start_edge ON
        start_edge.line_id = L2.line_id 
        and L2.fraction >= start_edge.fraction
      LEFT JOIN nav_edges AS end_edge ON
        end_edge.id = start_edge.forward_edge_id 
      ORDER BY 1 ASC, abs(L2.fraction - start_edge.fraction) ASC
      LIMIT 1
    )%";

		stringReplace(":point:", p.str(), sql);

		auto r = dbConnection->exec1(sql);

		TrailInfo ti;

		Json::Reader reader;
		Json::Value root;
		reader.parse(r["point"].as<std::string>(), root);

		ti.m_point = {root[1].asDouble(), root[0].asDouble()};

		ti.m_distance = r["distance"].as<double>();
		ti.m_fraction = r["fraction"].as<double>();
		ti.m_startEdgeId = r["start_edge_id"].as<int>();
		ti.m_endEdgeId = r["end_edge_id"].as<int>();
		ti.m_startFraction = r["start_fraction"].as<double>();
		ti.m_endFraction = r["end_fraction"].as<double>();
		ti.m_startNodeId = r["start_node"].as<int>();
		ti.m_endNodeId = r["end_node"].as<int>();
		ti.m_lineId = r["line_id"].as<int>();
		ti.m_forwardCost = r["forward_cost"].as<double>();
		ti.m_backwardCost = r["reverse_cost"].as<double>();

		return ti;
	}
	catch (const pqxx::pqxx_exception &e)
	{
		std::cerr << e.base().what() << std::endl;
		const pqxx::sql_error *s = dynamic_cast<const pqxx::sql_error*>(&e.base());
		if (s)
		{
			std::cerr << "Query was: " << s->query() << std::endl;
		}
	}
	catch (...)
	{
		std::cerr << "getTrailFromPoint error" << std::endl;
	}

	return {};
}


TrailInfo getLineFromPoint(const Point &point)
{
	try
	{
    auto dbConnection = std::make_shared<DBConnection>();
		std::stringstream p;

		p << std::setprecision(15) << "ST_Transform('SRID=4326;POINT(" << point.m_lng << " "
				<< point.m_lat << ")'::geometry, 3857)" << std::defaultfloat;

		std::string sql = R"%(
				SELECT 
          ST_Distance(L2.way, :point:) AS distance, 
          ST_AsGeoJSON(ST_Transform(ST_ClosestPoint(L2.way, :point:), 4326)) AS point, 
          ST_LineLocatePoint(L2.way, :point:) AS fraction, 
          L2.line_id, 
          L2.highway, 
          COALESCE(L2.name, '') as name, 
          COALESCE(L2.surface, '') as surface 
				FROM (
          SELECT * 
				  FROM planet_osm_route 
				  ORDER BY way <-> :point: 
				  LIMIT 10
        ) L2 
				ORDER BY 1 ASC 
				LIMIT 1 
      )%";

		stringReplace(":point:", p.str(), sql);

		auto r = dbConnection->exec1(sql);

		TrailInfo ti;

		Json::Reader reader;
		Json::Value root;
		reader.parse(r["point"].as<std::string>(), root);

		ti.m_point = {root["coordinates"][1].asDouble(),
				root["coordinates"][0].asDouble()};

		ti.m_distance = r["distance"].as<double>();
		ti.m_fraction = r["fraction"].as<double>();
		ti.m_lineId = r["line_id"].as<int>();
		ti.m_highway = r["highway"].as<std::string>();
		ti.m_name = r["name"].as<std::string>();
		ti.m_surface = r["surface"].as<std::string>();

		return ti;
	}
	catch (const pqxx::pqxx_exception &e)
	{
		std::cerr << e.base().what() << std::endl;
		const pqxx::sql_error *s = dynamic_cast<const pqxx::sql_error*>(&e.base());
		if (s)
		{
			std::cerr << "Query was: " << s->query() << std::endl;
		}
	}
	catch (...)
	{
		std::cerr << "getTrailFromPoint error" << std::endl;
	}

	return {};
}

void updateRouteElevation(
  int lineId)
{
    auto dbConnection = std::make_shared<DBConnection>();

		PreparedStatement updateRouteElevations (
        dbConnection,
        R"%(
				update planet_osm_route 
				set way2 = ST_SetSRID(ST_GeomFromGeoJSON($1), 3857) 
				where line_id = $2
        )%");

    PreparedStatement getRoute(
        dbConnection,
        R"%(
          select ST_AsGeoJSON(way2) AS way1, ST_AsGeoJSON(ST_TRANSFORM(way2, 4326)) as way2
          from planet_osm_route
          where line_id = $1
        )%");

    auto transaction = dbConnection->newTransaction ();

    auto r = transaction.exec1(getRoute, lineId);

    Json::Reader reader;
    Json::FastWriter fastWriter;
    std::map<int, std::string> updates;

    Json::Value root1;
    reader.parse(r["way1"].as<std::string>(), root1);

    Json::Value root2;
    reader.parse(r["way2"].as<std::string>(), root2);

    int i = 0;
    for (auto const &coord: root2["coordinates"])
    {
      root1["coordinates"][i][2] = Elevation::getInstance()->getElevation(LatLng(coord[1].asDouble(), coord[0].asDouble()));
      i++;
    }

    transaction.exec (updateRouteElevations, fastWriter.write(root1), lineId);

    transaction.commit();
}

void updateRouteElevations()
{
  try
  {
    auto dbConnection = std::make_shared<DBConnection>();
    int rowsUpdated {0};
    int maxLineId {0};

		PreparedStatement updateRouteElevations (
        dbConnection,
				"update planet_osm_route "
				"set way2 = ST_SetSRID(ST_GeomFromGeoJSON($1), 3857) "
				"where line_id = $2 ");

    for (;;)
    {
      std::string sql =
        "SELECT line_id, ST_AsGeoJSON(way) AS way1, ST_AsGeoJSON(ST_TRANSFORM(way, 4326)) as way2 "
        "FROM planet_osm_route "
        "WHERE line_id > " + std::to_string(maxLineId) + " "
        "ORDER BY line_id "
        "LIMIT 1000";

      std::cerr << "Querying rows to update..." << std::endl;
      auto r = dbConnection->exec(sql);

      if (r.empty())
      {
        break;
      }

      Json::Reader reader;
      Json::FastWriter fastWriter;
      std::map<int, std::string> updates;

      std::cerr << "Updating elevations..." << std::endl;
      for (auto const &row: r)
      {
        Json::Value root1;
        reader.parse(row["way1"].as<std::string>(), root1);

        Json::Value root2;
        reader.parse(row["way2"].as<std::string>(), root2);

        int i = 0;
        for (auto const &coord: root2["coordinates"])
        {
          root1["coordinates"][i][2] = Elevation::getInstance()->getElevation(LatLng(coord[1].asDouble(), coord[0].asDouble()));
          i++;
        }

        updates[row["line_id"].as<int>()] = fastWriter.write(root1);
      }

      std::cerr << "Updating rows..." << std::endl;
      auto transaction = dbConnection->newTransaction ();

      for (const auto &entry: updates)
      {
        int lineId = entry.first;
        transaction.exec (updateRouteElevations, entry.second, lineId);
        rowsUpdated++;

        if (lineId > maxLineId)
        {
          maxLineId = lineId;
        }

        if (rowsUpdated % 10000 == 0)
        {
          std::cerr << "Updated " << rowsUpdated << std::endl;
        }
      }

  		transaction.commit ();
    }

    std::cerr << "Completed updating route elevations" << std::endl;
	}
	catch (const pqxx::pqxx_exception &e)
	{
		std::cerr << e.base().what() << std::endl;
		const pqxx::sql_error *s = dynamic_cast<const pqxx::sql_error*>(&e.base());
		if (s)
		{
			std::cerr << "Query was: " << s->query() << std::endl;
		}
	}
  catch (const std::exception &e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }
	catch (...)
	{
		std::cerr << "updateRouteElevations error" << std::endl;
	}
}


void updateNavEdgeCosts(const std::vector<std::vector<int>> &areas)
{
	try
	{
    auto dbConnection = std::make_shared<DBConnection>();

		PreparedStatement updateEdgeCosts (
      dbConnection,
      R"%(
				update nav_edges
				set forward_cost = $1,
        reverse_cost = $2
				where id = $3
      )%"
    );

    PreparedStatement queryEdges(
      dbConnection,
      R"%(
        select
          e1.id AS edge1_id,
          e2.id AS edge2_id,
          e1.line_id,
          ST_AsGeoJSON(ST_SwapOrdinates(ST_Transform(ST_LineSubString(way2, e1.fraction, e2.fraction), 4326), 'xy'))::json->'coordinates' as line
        from planet_osm_route as route
        join nav_edges AS e1 on e1.line_id = route.line_id
        join nav_edges AS e2 on e2.reverse_edge_id = e1.id and e1.forward_edge_id = e2.id and e1.line_id = e2.line_id
        where e1.forward_edge_id != -1
      	and route.way2 && ST_SetSRID(ST_MakeBox2D(
          ST_Transform(ST_SetSRID(ST_MakePoint($1, $2), 4326), 3857),
          ST_Transform(ST_SetSRID(ST_MakePoint($1 + 1, $2 + 1), 4326), 3857)
        ), 3857)
      )%"
    );
  
    for (const auto &area: areas)
    {
      int lat = area[0];
      int lng = area[1];
      std::cerr << "Querying rows to update in (" << lat << ", " << lng << ")..." << std::endl;

      auto r = queryEdges.exec(lng, lat);

      if (r.empty())
      {
        continue;
      }

      struct Costs
      {
        double forwardCost {-1};
        double backwardCost {-1};
      };

      std::map<int, Costs> updates;

      std::cerr << "Updating costs..." << std::endl;

      Json::Reader reader;
      for (auto const &row: r)
      {
        double forwardCost;
        double backwardCost;

        Json::Value line;
        reader.parse(row["line"].as<std::string>(), line);

        std::tie(forwardCost, backwardCost) = computeCosts(line);
        
        updates[row["edge1_id"].as<int>()].forwardCost = forwardCost;
        updates[row["edge2_id"].as<int>()].backwardCost = backwardCost;
      }

      std::cerr << "Updating rows..." << std::endl;
      auto transaction = dbConnection->newTransaction ();

      for (const auto &entry: updates)
      {
        int id = entry.first;
        transaction.exec (updateEdgeCosts, entry.second.forwardCost, entry.second.backwardCost, id);
      }

  		transaction.commit ();
    }

    std::cerr << "Completed updating edge costs" << std::endl;
	}
	catch (const pqxx::pqxx_exception &e)
	{
		std::cerr << e.base().what() << std::endl;
		const pqxx::sql_error *s = dynamic_cast<const pqxx::sql_error*>(&e.base());
		if (s)
		{
			std::cerr << "Query was: " << s->query() << std::endl;
		}
	}
  catch (const std::exception &e)
  {
    std::cerr << "Error: " << e.what() << std::endl;
  }
	catch (...)
	{
		std::cerr << "updateRouteElevations error" << std::endl;
	}
}
