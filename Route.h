/*
 * routeDistance.h
 *
 *  Created on: Nov 3, 2019
 *      Author: richard
 */

#pragma once

#include "Point.h"
#include "DBConnection.h"
#include <vector>

class RoutePoint {
public:
  RoutePoint(const Point &point, int prevLineId, double prevFraction,
             int nextLineId, double nextFraction, const std::string &type,
             int sortOrder, const std::string &name)
      : m_point(point), m_prevLineId(prevLineId), m_prevFraction(prevFraction),
        m_nextLineId(nextLineId), m_nextFraction(nextFraction), m_type(type),
        m_sortOrder(sortOrder), m_name(name) {}

  Point m_point;
  int m_prevLineId{-1};
  double m_prevFraction{-1};
  int m_nextLineId{-1};
  double m_nextFraction{-1};
  std::string m_type;
  int m_sortOrder{-1};
  std::string m_name;
  std::vector<Point> m_trail;
};

class Route {
public:
  Route(int routeId);

  double getDistance();

private:
  std::tuple<std::vector<Point>, double>
  getTrailPoints(int lineId, double startFraction, double endFraction,
                bool getDistances, bool getElevations);

  double getTrailPointsBetweenAnchors(size_t anchorIndex, bool getDistances,
                                      bool getElevations);

  int m_routeId{-1};
  std::vector<RoutePoint> m_routePoints;

  std::shared_ptr<DBConnection> m_dbConnection;

  PreparedStatement m_queryTrailPoints;
  PreparedStatement m_reverseQueryTrailPoints;
};
