#pragma once

#include <memory>

class Node;

class EdgeEnd
{
public:

  int m_edgeId {-1};
  int m_nodeId {-1};
	double m_fraction {-1};

  // In the start edge end, cost is the forward cost.
  // In the end edge end, cost is the backward cost.
	double m_cost {-1};

  std::shared_ptr<Node> m_node;

private:
};

