#pragma once

#include "LatLng.h"

class LatLngBounds
{
public:
  LatLngBounds(double south, double west, double north, double east)
  :
    m_southWest(south, west),
    m_northEast(north, east)
  {
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

  LatLng m_southWest;
  LatLng m_northEast;

private:
};