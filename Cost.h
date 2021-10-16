#pragma once

#include "LatLng.h"
#include <tuple>


double haversineGreatCircleDistance(
  const LatLng &from,
  const LatLng &to,
	double earthRadius = 6378137);

double metersPerHourGet(double deltaH, double deltaX);

std::tuple<double, double> computeLineSubstringCost(
		int lineId, double startFraction,
		double endFraction);

std::tuple<double, double> computeCosts(const Json::Value &points);
