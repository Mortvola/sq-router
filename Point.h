/*
 * Point.h
 *
 *  Created on: Oct 26, 2019
 *      Author: richard
 */

#pragma once

#include <ostream>
#include <iomanip>
#include <json/json.h>
#include "LatLng.h"

class Point : public LatLng
{
public:

	Point () = default;

	Point(double lat, double lng)
	:
		LatLng (lat, lng)
	{
	}

	Point(double lat, double lng, double elevation)
	:
		LatLng (lat, lng),
    m_elevation (elevation)
	{
	}

	Point (const Json::Value &value)
	:
		LatLng (value["lat"].asDouble (), value["lng"].asDouble ())
	{
	}

	~Point () = default;

	Point (const Point &other) = default;
	Point (Point &&other) = default;
	Point &operator= (const Point &other) = default;
	Point &operator= (Point &&other) = default;

	Point &operator= (const Json::Value &value)
	{
		m_lat = value["point"]["lat"].asDouble ();
		m_lng = value["point"]["lng"].asDouble ();

		return *this;
	}

	operator Json::Value () const
	{
		Json::Value value;

		value["lat"] = m_lat;
		value["lng"] = m_lng;

		return value;
	}

	friend std::ostream& operator<<(std::ostream &os, const Point &point)
	{
		os << std::setprecision(15) << "[" << point.m_lat << ", " << point.m_lng << "]" << std::defaultfloat;

		return os;
	}

	double m_elevation {-1};
	double m_distance {-1};
};

