#pragma once

namespace gb
{
  struct EdgeUpdate
  {
    enum class Operation
    {
      update,
      insert
    };

    EdgeUpdate(Operation o, Edge *e)
    :
      operation(o),
      edge(e)
    {}

    Operation operation;
    Edge *edge;
  };
}
