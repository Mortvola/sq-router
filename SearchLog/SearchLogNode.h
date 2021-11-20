#pragma once

#include "LatLng.h"

class SearchLogNode
{
public:
  SearchLogNode(int nodeId, const LatLng &latlng, double cumulativeCost, double timeToEnd, bool preferredNode)
  :
    m_nodeId(nodeId),
    m_latlng(latlng),
    m_cumulativeCost(cumulativeCost),
    m_timeToEnd(timeToEnd),
    m_preferredNode(preferredNode)
  {
  }

  operator Json::Value() const
  {
    Json::Value n;

    n["nodeId"] = m_nodeId;
    n["latlng"] = m_latlng;
    n["cumulativeCost"] = m_cumulativeCost;
    n["timeToEnd"] = m_timeToEnd;
    n["preferred"] = m_preferredNode;

    return n;
  }

  int m_nodeId;
  LatLng m_latlng;
  double m_cumulativeCost;
  double m_timeToEnd;
  bool m_preferredNode;

private:
};
