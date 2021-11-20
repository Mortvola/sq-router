#include "SearchLog.h"
#include "SearchNode.h"
#include "SearchController.h"

SearchLog::SearchLog(SearchController &controller)
:
  m_controller(controller)
{
}

SearchLog::~SearchLog()
{
}

SearchLog::operator Json::Value () const
{
  Json::Value entries = Json::arrayValue;

  for (const auto &entry: m_entries)
  {
    entries.append(entry);
  }

  return entries;
}

