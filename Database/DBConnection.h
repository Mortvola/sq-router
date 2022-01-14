/*
 * DBConnection.h
 *
 *  Created on: Oct 27, 2019
 *      Author: richard
 */

#pragma once

#include "DBTransaction.h"
#include "PreparedStatement.h"
#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <pqxx/pqxx>
#include <string>
#include <tuple>
#include <vector>

void configDB(const std::string &username, const std::string &password,
              const std::string &host, const std::string &database);

class DBConnection : public std::enable_shared_from_this<DBConnection> {
public:
  DBConnection();

  void prepare(const std::string &stmtName, const std::string &statement);

  void unprepare(const std::string &stmtName);

  pqxx::result exec(const std::string &sql);

  template <typename... Args>
  pqxx::result exec (const std::string &sql, Args &&... args)
  {
    std::unique_lock<std::mutex> lock(m_dbAccess);

    pqxx::work query(DBConnection::connection ());

    return query.exec_params(sql, args...);
  }

  pqxx::row exec1(const std::string &sql);

  template <typename tupleType>
  void streamFrom(const std::string &tableName,
                  const std::vector<std::string> &columns, tupleType &row,
                  std::function<void()> callback) {
    pqxx::work query(DBConnection::connection());

    pqxx::stream_from stream{query, tableName, columns};

    while (stream >> row) {
      callback();
    }

    stream.complete();
  }

  DBTransaction newTransaction();

private:
  friend class PreparedStatement;
  friend class DBTransaction;

  template <typename... Args>
  pqxx::result execPrepared(const std::string &stmtName, Args &&... args) {
    std::unique_lock<std::mutex> lock(m_dbAccess);

    pqxx::work txn(DBConnection::connection());

    return txn.exec_prepared(stmtName, args...);
  }

  template <typename... Args>
  pqxx::row execPrepared1(const std::string &stmtName, Args &&... args) {
    std::unique_lock<std::mutex> lock(m_dbAccess);

    pqxx::work query(DBConnection::connection());

    return query.exec_prepared1(stmtName, args...);
  }

  pqxx::connection &connection();

  std::mutex m_dbAccess;
  std::shared_ptr<pqxx::connection> m_connection;
  std::vector<std::string> m_preparedStatements;
};

template <typename... Args> pqxx::result PreparedStatement::exec(Args &&... args) {
  return m_dbConnection->execPrepared(m_name, args...);
}

template <typename... Args> pqxx::row PreparedStatement::exec1(Args &&... args) {
  return m_dbConnection->execPrepared1(m_name, args...);
}
