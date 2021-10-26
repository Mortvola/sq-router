#pragma once

#include "LatLng.h"

class LatLngBounds
{
public:
  LatLngBounds()
  :
    m_southWest(0, 0),
    m_northEast(0, 0)
  {
  }

  LatLngBounds(double south, double west, double north, double east)
  :
    m_southWest(south, west),
    m_northEast(north, east)
  {
  }

	bool operator== (const LatLngBounds &other) const
	{
		return m_southWest == other.m_southWest && m_northEast == other.m_northEast;
	}

	bool operator!= (const LatLngBounds &other) const
	{
		return !operator==(other);
	}

  bool contains(const LatLng &latlng) const
  {
    return latlng.m_lat >= m_southWest.m_lat
      && latlng.m_lat <= m_northEast.m_lat
      && latlng.m_lng >= m_southWest.m_lng
      && latlng.m_lng <= m_northEast.m_lng;
  }

	friend std::ostream& operator<<(std::ostream &os, const LatLngBounds &latLngBounds)
	{
		os << std::setprecision(15) << latLngBounds.m_southWest << " - " << latLngBounds.m_northEast;

		return os;
	}

	operator Json::Value () const
	{
		Json::Value value;

		value["southWest"] = m_southWest;
		value["northEast"] = m_northEast;

		return value;
	}

  LatLng m_southWest;
  LatLng m_northEast;

private:
};
