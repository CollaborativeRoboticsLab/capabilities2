#pragma once

#include <memory>
#include <vector>
#include <string>

#include <sqlite3.h>
#include <yaml-cpp/yaml.h>

#include <capabilities2_server/models/interface.hpp>
#include <capabilities2_server/models/semantic_interface.hpp>
#include <capabilities2_server/models/provider.hpp>

namespace capabilities2_server
{

class CapabilitiesDB
{
public:
  CapabilitiesDB(const std::string db_file = "cache/capabilities.sqlite3") : db_(nullptr), db_file_(db_file)
  {
    // open db
    open();

    // create tables if they do not exists
    create_tables();
  }

  ~CapabilitiesDB()
  {
    close();
  }

  /* inserters */

  // insert interface
  void insert_interface(const models::interface_model_t& interface)
  {
    const std::string query = "INSERT INTO interfaces VALUES (" + interface.to_sql_values() + ")";
    exec(query);
  }

  // insert semantic interface
  void insert_semantic_interface(const models::semantic_interface_model_t& semantic_interface)
  {
    const std::string query = "INSERT INTO semantic_interfaces VALUES (" + semantic_interface.to_sql_values() + ")";
    exec(query);
  }

  // insert provider
  void insert_provider(const models::provider_model_t& provider)
  {
    const std::string query = "INSERT INTO providers VALUES (" + provider.to_sql_values() + ")";
    exec(query);
  }

  /* getters */

  // get interface
  models::interface_model_t get_interface(const std::string& name)
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM interfaces WHERE name = '" + name + "'";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      close();
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    models::interface_model_t interface;
    if (sqlite3_step(stmt) == SQLITE_ROW)
    {
      interface.header.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
      interface.header.version = sqlite3_column_int(stmt, 1);
      interface.header.type = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
      interface.header.description = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));
      interface.interface = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
    }

    sqlite3_finalize(stmt);
    return interface;
  }

  // get interfaces
  std::vector<models::interface_model_t> get_interfaces()
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM interfaces";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      close();
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    std::vector<models::interface_model_t> interfaces;
    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
      models::interface interface_model_t;
      interface.header.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
      interface.header.version = sqlite3_column_int(stmt, 1);
      interface.header.type = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
      interface.header.description = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));
      interface.interface = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4));
      interfaces.push_back(interface);
    }

    sqlite3_finalize(stmt);
    return interfaces;
  }

  // get semantic interface
  models::semantic_interface_model_t get_semantic_interface(const std::string& name)
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM semantic_interfaces WHERE name = '" + name + "'";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      close();
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    models::semantic_interface_model_t semantic_interface;
    if (sqlite3_step(stmt) == SQLITE_ROW)
    {
      semantic_interface.header.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
      semantic_interface.header.version = sqlite3_column_int(stmt, 1);
      semantic_interface.header.type = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
      semantic_interface.header.description = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));
      semantic_interface.redefines = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4)));
      semantic_interface.global_namespace = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5)));
      semantic_interface.remappings = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6)));
    }

    sqlite3_finalize(stmt);
    return semantic_interface;
  }

  // get semantic interfaces
  std::vector<models::semantic_interface_model_t> get_semantic_interfaces()
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM semantic_interfaces";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      close();
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    std::vector<models::semantic_interface_model_t> semantic_interfaces;
    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
      models::semantic_interface_model_t semantic_interface;
      semantic_interface.header.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
      semantic_interface.header.version = sqlite3_column_int(stmt, 1);
      semantic_interface.header.type = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
      semantic_interface.header.description = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));
      semantic_interface.redefines = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4)));
      semantic_interface.global_namespace = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5)));
      semantic_interface.remappings = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6)));
      semantic_interfaces.push_back(semantic_interface);
    }

    sqlite3_finalize(stmt);
    return semantic_interfaces;
  }

  // get provider
  models::provider_model_t get_provider(const std::string& name)
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM providers WHERE name = '" + name + "'";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      close();
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    models::provider_model_t provider;
    if (sqlite3_step(stmt) == SQLITE_ROW)
    {
      provider.header.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
      provider.header.version = sqlite3_column_int(stmt, 1);
      provider.header.type = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
      provider.header.description = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));
      provider.implements = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4)));
      provider.runner = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5)));
    }

    sqlite3_finalize(stmt);
    return provider;
  }

  // get providers
  std::vector<models::provider_model_t> get_providers()
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM providers";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      close();
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    std::vector<models::provider_model_t> providers;
    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
      models::provider_model_t provider;
      provider.header.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
      provider.header.version = sqlite3_column_int(stmt, 1);
      provider.header.type = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
      provider.header.description = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));
      provider.implements = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4)));
      provider.runner = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5)));
      providers.push_back(provider);
    }

    sqlite3_finalize(stmt);
    return providers;
  }

  /* updaters */

  // update interface
  void update_interface(const models::interface_model_t& interface)
  {
    const std::string query =
        "UPDATE interfaces SET " + interface.to_sql_values() + " WHERE name = '" + interface.header.name + "'";
    exec(query);
  }

  // update semantic interface
  void update_semantic_interface(const models::semantic_interface_model_t& semantic_interface)
  {
    const std::string query = "UPDATE semantic_interfaces SET " + semantic_interface.to_sql_values() +
                              " WHERE name = '" + semantic_interface.header.name + "'";
    exec(query);
  }

  // update provider
  void update_provider(const models::provider_model_t& provider)
  {
    const std::string query =
        "UPDATE providers SET " + provider.to_sql_values() + " WHERE name = '" + provider.header.name + "'";
    exec(query);
  }

  /* foreign keys */

  // get providers by interface
  std::vector<models::provider_model_t> get_providers_by_interface(const std::string& interface_name)
  {
    // could be a semantic interface or an interface
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM providers WHERE implements = '" + interface_name + "'";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    std::vector<models::provider_model_t> providers;
    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
      models::provider_model_t provider;
      provider.header.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
      provider.header.version = sqlite3_column_int(stmt, 1);
      provider.header.type = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
      provider.header.description = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));
      provider.implements = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4)));
      provider.runner = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5)));
      providers.push_back(provider);
    }
  }

  // get semantic interfaces by interface
  std::vector<models::semantic_interface_model_t>
  get_semantic_interfaces_by_interface(const std::string& interface_name)
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM semantic_interfaces WHERE redefines = '" + interface_name + "'";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    std::vector<models::semantic_interface_model_t> semantic_interfaces;
    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
      models::semantic_interface_model_t semantic_interface;
      semantic_interface.header.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
      semantic_interface.header.version = sqlite3_column_int(stmt, 1);
      semantic_interface.header.type = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
      semantic_interface.header.description = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));
      semantic_interface.redefines = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4)));
      semantic_interface.global_namespace = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5)));
      semantic_interface.remappings = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6)));
      semantic_interfaces.push_back(semantic_interface);
    }
  }

protected:
  void open()
  {
    const int exit = sqlite3_open(db_file_.c_str(), &db_);
    if (exit != SQLITE_OK)
    {
      sqlite3_close(db_);
      throw std::runtime_error("Error opening database: " + std::string(sqlite3_errmsg(db_)));
    }
  }

  void close()
  {
    // close if the database is open
    if (db_ != nullptr)
    {
      sqlite3_close(db_);
      db_ = nullptr;
    }
  }

  void exec(const std::string& query)
  {
    char* error_message;
    const int exit = sqlite3_exec(db_, query.c_str(), nullptr, nullptr, &error_message);
    if (exit != SQLITE_OK)
    {
      sqlite3_free(error_message);
      throw std::runtime_error("Error executing query: " + std::string(sqlite3_errmsg(db_)));
    }
  }

  void create_tables()
  {
    // create tables if they do not exists
    const std::string create_interface_table =
        "CREATE TABLE IF NOT EXISTS interfaces (" + models::interface_model_t::to_sql_table() + ")";

    const std::string create_semantic_interface_table =
        "CREATE TABLE IF NOT EXISTS semantic_interfaces (" + models::semantic_interface_model_t::to_sql_table() + ")";

    const std::string create_provider_table =
        "CREATE TABLE IF NOT EXISTS providers (" + models::provider_model_t::to_sql_table() + ")";

    // XXX TODO: create meta table
    // meta table
    const std::string create_meta_table = "CREATE TABLE IF NOT EXISTS meta (key TEXT NOT NULL, value TEXT NOT NULL)";

    // execute create queries
    exec(create_interface_table);
    exec(create_semantic_interface_table);
    exec(create_provider_table);
  }

private:
  std::string db_file_;
  sqlite3* db_;
};

}  // namespace capabilities2_server
