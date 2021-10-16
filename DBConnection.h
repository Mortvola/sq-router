/*
 * DBConnection.h
 *
 *  Created on: Oct 27, 2019
 *      Author: richard
 */

#pragma once

#include <string>
#include <memory>
#include <vector>
#include <algorithm>
#include <pqxx/pqxx>
#include <mutex>
#include <tuple>
#include <functional>

class PreparedStatement;


class DBTransaction
{
public:

	DBTransaction (const std::shared_ptr<pqxx::connection> &connection)
	:
		m_connection (connection),
		m_transaction (std::make_unique<pqxx::work>(*connection))
	{
	}

	DBTransaction (DBTransaction &&other) = default;

	template<typename ... Args>
	pqxx::result exec (PreparedStatement &stmt, Args &&... args);

	template<typename ... Args>
	pqxx::row exec1 (PreparedStatement &stmt, Args &&... args);

  pqxx::result exec (const std::string &sql);

  pqxx::row exec1 (const std::string &sql);

	void commit ()
	{
		m_transaction->commit ();
	}

private:

	std::shared_ptr<pqxx::connection> m_connection;
	std::unique_ptr<pqxx::work> m_transaction;
};

void configDB(const std::string &username, const std::string &password, const std::string &host, const std::string &database);

class DBConnection
{
public:

  DBConnection();

	void prepare (const std::string &stmtName, const std::string &statement);

	void unprepare (const std::string &stmtName);

	pqxx::result exec (const std::string &sql);

	pqxx::row exec1 (const std::string &sql);

  template<typename tupleType>
  void streamFrom(const std::string &tableName, const std::vector<std::string> &columns, tupleType &row, std::function<void()> callback)
  {
    pqxx::work query(DBConnection::connection ());

    pqxx::stream_from stream {query, tableName, columns};

    while (stream >> row)
    {
      callback();
    }

    stream.complete();
  }

	DBTransaction newTransaction ();

private:

  friend class PreparedStatement;

  template<typename ... Args>
  pqxx::result execPrepared (const std::string &stmtName, Args &&... args)
  {
    std::unique_lock<std::mutex> lock(m_dbAccess);

    pqxx::work txn(DBConnection::connection ());

    return txn.exec_prepared (stmtName, args...);
  }

  template<typename ... Args>
  pqxx::row execPrepared1 (const std::string &stmtName, Args &&... args)
  {
    std::unique_lock<std::mutex> lock(m_dbAccess);

    pqxx::work query(DBConnection::connection ());

    return query.exec_prepared1 (stmtName, args...);
  }

	pqxx::connection &connection ();

	std::mutex m_dbAccess;
	std::shared_ptr<pqxx::connection> m_connection;
	std::vector<std::string> m_preparedStatements;
};

class PreparedStatement
{
public:
  PreparedStatement(
    std::shared_ptr<DBConnection> dbConnection,
    const std::string &query)
  :
    m_dbConnection(dbConnection),
    m_name("Query" + std::to_string(m_counter++))
  {
    m_dbConnection->prepare(m_name, query);
  }

  ~PreparedStatement()
  {
    m_dbConnection->unprepare(m_name);
  }

  template<typename ... Args>
  pqxx::result exec (Args &&... args)
  {
    return m_dbConnection->execPrepared(m_name, args...);
  }

  template<typename ... Args>
  pqxx::row exec1 (Args &&... args)
  {
    return m_dbConnection->execPrepared1(m_name, args...);
  }

private:

  friend class DBTransaction;

  std::shared_ptr<DBConnection> m_dbConnection;

  static int m_counter;
  std::string m_name;
};

template<typename ... Args>
pqxx::result DBTransaction::exec (PreparedStatement &stmt, Args &&... args)
{
  return m_transaction->exec_prepared (stmt.m_name, args...);;
}

template<typename ... Args>
pqxx::row DBTransaction::exec1 (PreparedStatement &stmt, Args &&... args)
{
  return m_transaction->exec_prepared1 (stmt.m_name, args...);
}
