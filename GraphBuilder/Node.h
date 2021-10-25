#pragma once

#include "Edge.h"
#include <string>
#include <vector>

namespace gb 
{
  class Node
  {
  public:
    Node()
    {
    };

    Node(
      int id,
      const std::string &w,
      bool sameQuadrangle
    )
    :
      m_nodeId(id),
      m_way(w),
      m_sameQuadrangle(sameQuadrangle)
    {};

    std::vector<Edge> m_edges;

    int m_nodeId {-1};
    std::string m_way;
    bool m_sameQuadrangle {false};
  };
}
