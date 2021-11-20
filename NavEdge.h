#pragma once

class Edge;

class NavEdge
{
public:
  int m_edgeId;
  int m_nodeId;
  int m_otherEdgeId[2];

  std::shared_ptr<Edge> m_edge[2];

private:

};
