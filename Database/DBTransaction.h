#pragma once

#include "PreparedStatement.h"
#include <pqxx/pqxx>
#include <memory>

class DBConnection;
class PreparedStatement;

class DBTransaction {
public:
  DBTransaction(const std::shared_ptr<DBConnection> &connection);

  DBTransaction(DBTransaction &&other) = default;

  template <typename... Args>
  pqxx::result exec(PreparedStatement &stmt, Args &&... args);

  template <typename... Args>
  pqxx::row exec1(PreparedStatement &stmt, Args &&... args);

  pqxx::result exec(const std::string &sql);

  template <typename... Args>
  pqxx::result exec(const std::string &sql, Args &&... args);

  pqxx::row exec1(const std::string &sql);

  template <typename... Args>
  pqxx::row exec1(const std::string &sql, Args &&... args);

  void commit() { m_transaction->commit(); }

  std::shared_ptr<DBConnection> connection() { return m_dbConnection; }

private:
  std::shared_ptr<DBConnection> m_dbConnection;
  std::unique_ptr<pqxx::work> m_transaction;
};

template <typename... Args>
pqxx::result DBTransaction::exec(PreparedStatement &stmt, Args &&... args) {
  return m_transaction->exec_prepared(stmt.m_name, args...);
  ;
}

template <typename... Args>
pqxx::result DBTransaction::exec(const std::string &sql, Args &&... args) {
  return m_transaction->exec_params(sql, args...);
  ;
}

template <typename... Args>
pqxx::row DBTransaction::exec1(PreparedStatement &stmt, Args &&... args) {
  return m_transaction->exec_prepared1(stmt.m_name, args...);
}

template <typename... Args>
pqxx::row DBTransaction::exec1(const std::string &sql, Args &&... args) {
  return m_transaction->exec_params1(sql, args...);
  ;
}

