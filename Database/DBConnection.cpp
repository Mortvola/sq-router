#include "./Database.h"
#include <iostream>

std::string dbConnectionString;

void configDB(const std::string &username, const std::string &password, const std::string &host, const std::string &database)
{
  dbConnectionString = 
      std::string("postgresql://")
      + username + ":"
      + password + "@"
      + host + ":5432/"
      + database;
}

DBConnection::DBConnection()
{
  m_connection = std::make_shared<pqxx::connection>(dbConnectionString);
}

void DBConnection::prepare (const std::string &stmtName, const std::string &statement)
{
  std::unique_lock<std::mutex> lock(m_dbAccess);

  m_connection->prepare (stmtName, statement);
}

void DBConnection::unprepare (const std::string &stmtName)
{
  try
  {
    m_connection->unprepare(stmtName);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
}


pqxx::result DBConnection::exec (const std::string &sql)
{
  std::unique_lock<std::mutex> lock(m_dbAccess);

  pqxx::work query(DBConnection::connection ());

  return query.exec(sql);
}

pqxx::row DBConnection::exec1 (const std::string &sql)
{
  std::unique_lock<std::mutex> lock(m_dbAccess);

  pqxx::work query(DBConnection::connection ());

  return query.exec1(sql);
}

DBTransaction DBConnection::newTransaction ()
{
  return {shared_from_this()};
}

pqxx::connection &DBConnection::connection ()
{
  return *m_connection;
}
