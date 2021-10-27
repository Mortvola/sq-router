#pragma once

#include "DBConnection.h"
#include <pqxx/pqxx>
#include <memory>
#include <string>

class DBConnection;
class DBTransaction;

class PreparedStatement {
public:
  PreparedStatement(std::shared_ptr<DBConnection> dbConnection,
                    const std::string &query);

  PreparedStatement(DBTransaction &transaction, const std::string &query);

  ~PreparedStatement();

  template <typename... Args> pqxx::result exec(Args &&... args);

  template <typename... Args> pqxx::row exec1(Args &&... args);

private:
  friend class DBTransaction;

  std::shared_ptr<DBConnection> m_dbConnection;

  static int m_counter;
  std::string m_name;
};
