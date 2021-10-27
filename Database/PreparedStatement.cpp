#include "./Database.h"

int PreparedStatement::m_counter = 0;

PreparedStatement::PreparedStatement(std::shared_ptr<DBConnection> dbConnection,
                  const std::string &query)
    : m_dbConnection(dbConnection),
      m_name("Query" + std::to_string(m_counter++)) {
  m_dbConnection->prepare(m_name, query);
}

PreparedStatement::PreparedStatement(DBTransaction &transaction, const std::string &query)
    : m_dbConnection(transaction.connection()),
      m_name("Query" + std::to_string(m_counter++)) {
  m_dbConnection->prepare(m_name, query);
}

PreparedStatement::~PreparedStatement() { m_dbConnection->unprepare(m_name); }


