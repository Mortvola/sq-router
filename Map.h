/*
 * Map.h
 *
 *  Created on: Nov 3, 2019
 *      Author: richard
 */

#pragma once

#include "Point.h"
#include <json/json.h>
#include "TrailInfo.h"

TrailInfo getTrailFromPoint(const Point &point);
TrailInfo getLineFromPoint(const Point &point);
void updateRouteElevations();
void updateRouteElevation(int lineId);
void updateNavEdgeCosts(const std::vector<std::vector<int>> &areas);
