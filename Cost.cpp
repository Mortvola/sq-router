#include "Cost.h"
#include "DBConnection.h"
#include "Elevation.h"
#include "Point.h"
#include <sstream>
#include <cmath>
#include <iomanip>
#include <jsoncpp/json/json.h>
#include <iostream>


double metersPerHourGet(double deltaH, double deltaX)
{
	// This formula was defined by Tobler
	// On flat ground the formula works out to about 5 km/h.
	auto metersPerHour = 6000 * std::pow(2.71828, -3.5 * std::abs(deltaH / deltaX + 0.05));

	// Make sure the minimum speede is 1/2 kilometer per hour.
	// Sometimes elevation data is wrong and it may look like one is
	// climbing up an extremely steep cliff.

	return std::max(metersPerHour, 500.0);
}


double deg2rad(double degree)
{
	return (degree * M_PI / 180.0);
}

double haversineGreatCircleDistance(
  const LatLng &from,
  const LatLng &to,
	double earthRadius)
{
	// convert from degrees to radians
	auto latFrom = deg2rad(from.m_lat);
	auto lonFrom = deg2rad(from.m_lng);
	auto latTo = deg2rad(to.m_lat);
	auto lonTo = deg2rad(to.m_lng);

	auto latDelta = latTo - latFrom;
	auto lonDelta = lonTo - lonFrom;

	auto angle = 2
			* asin(
					sqrt(
							std::pow(sin(latDelta / 2.0), 2.0)
									+ cos(latFrom) * cos(latTo)
											* std::pow(sin(lonDelta / 2.0), 2.0)));

	return angle * earthRadius;
}

double haversineGreatCircleDistance(
  const Json::Value &from,
  const Json::Value &to,
	double earthRadius = 6378137)
{
	// convert from degrees to radians
	auto latFrom = deg2rad(from[0].asDouble());
	auto lonFrom = deg2rad(from[1].asDouble());
	auto latTo = deg2rad(to[0].asDouble());
	auto lonTo = deg2rad(to[1].asDouble());

	auto latDelta = latTo - latFrom;
	auto lonDelta = lonTo - lonFrom;

	auto angle = 2
			* asin(
					sqrt(
							std::pow(sin(latDelta / 2.0), 2.0)
									+ cos(latFrom) * cos(latTo)
											* std::pow(sin(lonDelta / 2.0), 2.0)));

	return angle * earthRadius;
}

std::tuple<double, double> computeCosts(const std::vector<Point> &points)
{
	double forwardCost {};
	double backwardCost {};

	return std::tuple<double, double>(forwardCost, backwardCost);
}

std::tuple<double, double> computeCosts(const Json::Value &points)
{
	double forwardCost {};
	double backwardCost {};

	if (points.size () >= 2)
	{
		auto ele1 = points[0][2].asDouble();
    
    if (ele1 == -1)
    {
      ele1 = Elevation::getInstance()->getElevation(points[0]);
    }

		for (size_t i = 1; i < points.size(); i++)
		{
			auto deltaX = haversineGreatCircleDistance(
        points[static_cast<int>(i - 1)],
        points[static_cast<int>(i)]);

			if (deltaX != 0)
			{
        auto ele2 = points[static_cast<int>(i)][2].asDouble();

        if (ele2 == -1)
        {
  				ele2 = Elevation::getInstance()->getElevation(points[static_cast<int>(i)]);
        }

				auto deltaH = ele2 - ele1;

				forwardCost += deltaX / metersPerHourGet(deltaH, deltaX);
				backwardCost += deltaX / metersPerHourGet(-deltaH, deltaX);

				ele1 = ele2;
			}
		}
	}

	return std::tuple<double, double>(forwardCost, backwardCost);
}


std::tuple<double, double> computeLineSubstringCost(
	int lineId,
	double startFraction,
	double endFraction)
{
	std::stringstream sql;

  auto dbConnection = std::make_shared<DBConnection>();

	PreparedStatement queryLineSubString (
    dbConnection,
    R"%(
			select
        ST_AsGeoJSON(
          ST_SwapOrdinates(ST_Transform(
            ST_LineSubString(way2, $2, $3),
            4326
          ), 'xy')
        ) as line 
			from planet_osm_route l1 
			where l1.line_id = $1
    )%"
  );

	auto r = queryLineSubString.exec1(
		lineId,
		startFraction,
		endFraction);

	Json::Reader reader;
	Json::Value line;
	reader.parse(r["line"].as<std::string>(), line);

  if (line["type"].asString() == "LineString")
  {
  	return computeCosts(line["coordinates"]);
  }

  return {0, 0};
}
