#include "./Database.h"

DBTransaction::DBTransaction (const std::shared_ptr<DBConnection> &connection)
:
  m_dbConnection (connection),
  m_transaction (std::make_unique<pqxx::work>(connection->connection()))
{
}

pqxx::result DBTransaction::exec (const std::string &sql)
{
  return m_transaction->exec(sql);
}

pqxx::row DBTransaction::exec1 (const std::string &sql)
{
  return m_transaction->exec1(sql);
}
