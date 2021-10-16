#pragma once

#include "Point.h"
#include <vector>
#include <memory>

class SearchController;
class SearchNode;

class SearchLogEntry {
public:
  enum class SearcherState
  {
    Unknown = -1,
    Active,
    Inactive,
  };

  SearchLogEntry(
    const std::string &type,
    int searcherId,
    SearcherState searcherState,
    int search,
    int fromSearcherId,
    const std::shared_ptr<SearchNode> &node,
    bool sharedNode,
    const SearchController &controller,
    double totalCost);

  SearchLogEntry(
    const std::string &type,
    int searcherId,
    SearcherState searcherState,
    int search,
    int fromSearcherId,
    const SearchController &controller,
    double totalCost);

  operator Json::Value() const;

  int m_entryID {0};
  std::string m_type;
  int m_searcherId{};
  SearcherState m_searcherState {SearcherState::Unknown};
  int m_search{};
  int m_fromSearcherId{};
  int m_nodeId{};
  bool m_sharedNode{false};
  Point m_point;
  double m_cost{};
  double m_costToEnd{};
  double m_bestCost{};
  int m_edgeCount{};
  double m_totalCost;

private:

  std::string searchStateString() const
  {
    switch (m_searcherState)
    {
      case SearcherState::Unknown:
        return "Unknown";
      case SearcherState::Active:
        return "Active";
      case SearcherState::Inactive:
        return "Inactive";
    }

    return "Unknown";
  }
};
