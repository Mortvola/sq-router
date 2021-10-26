#pragma once

#include <iostream>
#include <iomanip>
#include <jsoncpp/json/json.h>


class LatLng
{
public:

	LatLng () = default;

	LatLng (double lat, double lng)
	:
		m_lat (lat),
		m_lng (lng)
	{
	}

	LatLng (const Json::Value &value)
	{
    if (value.isArray()) {
      m_lat = value[0].asDouble();
      m_lng = value[1].asDouble();
    }
    else {
      m_lat = value["lat"].asDouble ();
      m_lng = value["lng"].asDouble ();
    }
	}

	bool operator== (const LatLng &other) const
	{
		return m_lat == other.m_lat && m_lng == other.m_lng;
	}

	friend std::ostream& operator<<(std::ostream &os, const LatLng &latLng)
	{
		os << std::setprecision(15) <<
			"(" << latLng.m_lat <<
			", " << latLng.m_lng <<
			")";

		return os;
	}

	operator Json::Value () const
	{
		Json::Value value;

		value["lat"] = m_lat;
		value["lng"] = m_lng;

		return value;
	}

	double m_lat {0};
	double m_lng {0};
};


