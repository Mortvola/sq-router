/*
 * route.cpp
 *
 *  Created on: Nov 3, 2019
 *      Author: richard
 */

#include "Route.h"
#include "Cost.h"
#include "./Database/DBConnection.h"
#include "Elevation.h"
#include "Map.h"
#include <iostream>
#include <json/json.h>
#include <pqxx/pqxx>
#include <string>
#include <tuple>

Route::Route(int routeId)
:
  m_routeId(routeId),
  m_dbConnection(std::make_shared<DBConnection>()),
  m_queryTrailPoints (
    m_dbConnection,
    "select ST_AsGeoJSON(ST_Transform(ST_LineSubstring "
    "(way, $2, $3), 4326)) AS line "
    "from planet_osm_route line "
    "where line_id = $1 "
    "limit 1"),
  m_reverseQueryTrailPoints (
    m_dbConnection,
    "select ST_AsGeoJSON(ST_Transform(ST_LineSubstring "
    "(ST_Reverse(way), $2, $3), 4326)) AS line "
    "from planet_osm_route line "
    "where line_id = $1 "
    "limit 1")
{
  PreparedStatement queryAnchors (
      m_dbConnection,
      "select lat, lng, COALESCE(prev_line_id, -1) AS prev_line_id, "
      "COALESCE(prev_fraction, -1) AS prev_fraction, "
      "	COALESCE(next_line_id, -1) AS next_line_id, COALESCE(next_fraction, "
      "-1) AS next_fraction, "
      "	COALESCE(type, '') AS type, type_id, sort_order, COALESCE(name, '') AS "
      "name "
      "from route_point rp "
      "where rp.hike_id = $1 "
      "order by sort_order asc");

  auto anchors = queryAnchors.exec(m_routeId);

  for (const auto &anchorRow : anchors) {
    m_routePoints.emplace_back(
        Point(anchorRow["lat"].as<double>(), anchorRow["lng"].as<double>()),
        anchorRow["prev_line_id"].as<int>(),
        anchorRow["prev_fraction"].as<double>(),
        anchorRow["next_line_id"].as<int>(),
        anchorRow["next_fraction"].as<double>(),
        anchorRow["type"].as<std::string>(), anchorRow["sort_order"].as<int>(),
        anchorRow["name"].as<std::string>());
  }
}

double Route::getDistance() {
  std::cerr << "getting distance\n";

  double distance{0};

  if (m_routePoints.size() > 0) {
    for (size_t i = 0; i < m_routePoints.size() - 1; i++) {
      distance += getTrailPointsBetweenAnchors(i, true, false);
    }
  }

  return distance;
}

std::tuple<std::vector<Point>, double>
Route::getTrailPoints(int lineId, double startFraction, double endFraction,
               bool getDistances, bool getElevations) {
  std::string queryName = "queryTrailPoints";
  pqxx::row points;

  if (startFraction > endFraction)
  {
    startFraction = 1 - startFraction;
    endFraction = 1 - endFraction;

    points = m_reverseQueryTrailPoints.exec1 (lineId, startFraction, endFraction);
  }
  else
  {
    points = m_queryTrailPoints.exec1(lineId, startFraction, endFraction);
  }

  Json::Reader reader;
  Json::Value root;
  reader.parse(points["line"].as<std::string>(), root);

  std::vector<Point> line;
  double distance{0};
  Elevation elevation;

  if (getDistances) {
    // We discard the first and last points because they are represented
    // by the anchors.
    for (size_t i = 1; i < root["coordinates"].size() - 1; i++) {
      if (i == 1) {
        auto &prevValue =
            root["coordinates"][static_cast<Json::Value::ArrayIndex>(0)];
        Point prevPoint(prevValue[1].asDouble(), prevValue[0].asDouble());

        auto &value =
            root["coordinates"][static_cast<Json::Value::ArrayIndex>(1)];

        line.emplace_back(value[1].asDouble(), value[0].asDouble());

        line[0].m_distance = haversineGreatCircleDistance(prevPoint, line[0]);
        distance += line[0].m_distance;

        if (getElevations) {
          line[0].m_elevation = elevation.getElevation(line[0]);
        }
      } else {
        auto &value =
            root["coordinates"][static_cast<Json::Value::ArrayIndex>(i)];

        line.emplace_back(value[1].asDouble(), value[0].asDouble());

        line[i - 1].m_distance =
            haversineGreatCircleDistance(line[i - 2], line[i - 1]);
        distance += line[i - 1].m_distance;

        if (getElevations) {
          line[i - 1].m_elevation = elevation.getElevation(line[i - 1]);
        }
      }
    }
  } else {
    for (size_t i = 1; i < root["coordinates"].size() - 1; i++) {
      auto &value =
          root["coordinates"][static_cast<Json::Value::ArrayIndex>(i)];

      line.emplace_back(value[1].asDouble(), value[0].asDouble());

      if (getElevations) {
        line[i - 1].m_elevation = elevation.getElevation(line[i - 1]);
      }
    }
  }

  return std::tuple<std::vector<Point>, double>(line, distance);
}

double Route::getTrailPointsBetweenAnchors(size_t anchorIndex,
                                           bool getDistances,
                                           bool getElevations) {
  auto &routePoint1 = m_routePoints[anchorIndex];
  auto &routePoint2 = m_routePoints[anchorIndex + 1];

  if (routePoint1.m_nextLineId == -1 || routePoint2.m_prevLineId == -1 ||
      routePoint1.m_nextLineId != routePoint2.m_prevLineId) {
    //		if (routePoint1.m_nextEdgeId == -1)
    //		{
    //			auto result = getTrailFromPoint(routePoint1.m_point);
    //		}

    throw std::runtime_error("invalid route points");
  }

  double distance{0};

  std::tie(routePoint1.m_trail, distance) =
      getTrailPoints(routePoint1.m_nextLineId, routePoint1.m_nextFraction,
                     routePoint2.m_prevFraction, getDistances, getElevations);

  if (getDistances && routePoint1.m_trail.size() > 0) {
    // Add in the distance between the last point of the previous trail and the
    // second anchor.
    distance += haversineGreatCircleDistance(
        routePoint1.m_trail[routePoint1.m_trail.size() - 1],
        routePoint2.m_point);
  }

  return distance;
}
