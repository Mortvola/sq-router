#pragma once

namespace gb
{
  class Edge
  {
  public:
    Edge (int lid, int index, double f)
    :
      lineId(lid),
      pointIndex(index),
      fraction(f)
    {}

    Edge (const Edge &other) = default;
    Edge (Edge &&other) = default;
    Edge &operator=(const Edge &other) = default;
    Edge &operator=(Edge &&other) = default;

    int id {-1};
    int lineId {-1};
    int pointIndex {-1};
    int nodeId {-1};
    double fraction {0};
    double forwardCost {0};
    double reverseCost {0};
    int forwardEdgeId {-1};
    int reverseEdgeId {-1};

  private:
  };

  bool operator==(const Edge &a, const Edge &b);
  bool operator<(const Edge &a, const Edge &b);
}
