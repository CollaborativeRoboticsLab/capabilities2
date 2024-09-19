#pragma once

#include <memory>
#include <vector>
#include <string>

#include <sqlite3.h>
#include <yaml-cpp/yaml.h>

#include <capabilities2_server/db_base.hpp>
#include <capabilities2_server/models/interface.hpp>
#include <capabilities2_server/models/semantic_interface.hpp>
#include <capabilities2_server/models/provider.hpp>
#include <capabilities2_server/models/running.hpp>
#include <capabilities2_server/models/run_config.hpp>

namespace capabilities2_server
{

class CapabilitiesDB : public DBBase
{
public:
  CapabilitiesDB(const std::string db_file = "cache/capabilities.sqlite3") : db_file_(db_file), db_(nullptr)
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
  virtual void insert_interface(const models::interface_model_t& interface) override
  {
    const std::string query = "INSERT INTO interfaces VALUES (" + interface.to_sql_values() + ")";
    exec(query);
  }

  // insert semantic interface
  virtual void insert_semantic_interface(const models::semantic_interface_model_t& semantic_interface) override
  {
    const std::string query = "INSERT INTO semantic_interfaces VALUES (" + semantic_interface.to_sql_values() + ")";
    exec(query);
  }

  // insert provider
  virtual void insert_provider(const models::provider_model_t& provider) override
  {
    const std::string query = "INSERT INTO providers VALUES (" + provider.to_sql_values() + ")";
    exec(query);
  }

  /* getters */

  // get interface
  virtual models::interface_model_t get_interface(const std::string& name) override
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM interfaces WHERE name = '" + name + "'";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    models::interface_model_t interface;
    if (sqlite3_step(stmt) == SQLITE_ROW)
    {
      interface = to_interface(stmt);
    }

    sqlite3_finalize(stmt);
    return interface;
  }

  // get interfaces
  virtual std::vector<models::interface_model_t> get_interfaces() override
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM interfaces";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    std::vector<models::interface_model_t> interfaces;
    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
      models::interface_model_t interface = to_interface(stmt);
      interfaces.push_back(interface);
    }

    sqlite3_finalize(stmt);
    return interfaces;
  }

  // get semantic interface
  virtual models::semantic_interface_model_t get_semantic_interface(const std::string& name) override
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM semantic_interfaces WHERE name = '" + name + "'";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    models::semantic_interface_model_t semantic_interface;
    if (sqlite3_step(stmt) == SQLITE_ROW)
    {
      semantic_interface = to_semantic_interface(stmt);
    }

    sqlite3_finalize(stmt);
    return semantic_interface;
  }

  // get semantic interfaces
  virtual std::vector<models::semantic_interface_model_t> get_semantic_interfaces() override
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM semantic_interfaces";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    std::vector<models::semantic_interface_model_t> semantic_interfaces;
    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
      models::semantic_interface_model_t semantic_interface = to_semantic_interface(stmt);
      semantic_interfaces.push_back(semantic_interface);
    }

    sqlite3_finalize(stmt);
    return semantic_interfaces;
  }

  // get provider
  virtual models::provider_model_t get_provider(const std::string& name) override
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM providers WHERE name = '" + name + "'";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    models::provider_model_t provider;
    if (sqlite3_step(stmt) == SQLITE_ROW)
    {
      provider = to_provider(stmt);
    }

    sqlite3_finalize(stmt);
    return provider;
  }

  // get providers
  virtual std::vector<models::provider_model_t> get_providers() override
  {
    sqlite3_stmt* stmt;
    const std::string query = "SELECT * FROM providers";
    const int exit = sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr);
    if (exit != SQLITE_OK)
    {
      sqlite3_finalize(stmt);
      throw std::runtime_error("Error preparing query: " + std::string(sqlite3_errmsg(db_)));
    }

    std::vector<models::provider_model_t> providers;
    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
      models::provider_model_t provider = to_provider(stmt);
      providers.push_back(provider);
    }

    sqlite3_finalize(stmt);
    return providers;
  }

  /**
   * @brief Get remappable model by name
   *
   * remappable models are either semantic interfaces or providers
   *
   * @param name
   * @return models::remappable_base_t
   */
  virtual models::remappable_base_t get_remappable(const std::string& name) override
  {
    // check if it is a semantic interface
    models::semantic_interface_model_t semantic_interface = get_semantic_interface(name);
    if (!semantic_interface.header.name.empty())
    {
      return semantic_interface;
    }

    // check if it is a provider
    models::provider_model_t provider = get_provider(name);
    if (!provider.header.name.empty())
    {
      return provider;
    }

    // return empty remappable model
    return models::remappable_base_t();
  }

  /* updaters */

  // update interface
  virtual void update_interface(const models::interface_model_t& interface) override
  {
    const std::string query =
        "UPDATE interfaces SET " + interface.to_sql_values() + " WHERE name = '" + interface.header.name + "'";
    exec(query);
  }

  // update semantic interface
  virtual void update_semantic_interface(const models::semantic_interface_model_t& semantic_interface) override
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
  virtual std::vector<models::provider_model_t> get_providers_by_interface(const std::string& interface_name) override
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
      models::provider_model_t provider = to_provider(stmt);
      providers.push_back(provider);
    }

    return providers;
  }

  // get semantic interfaces by interface
  virtual std::vector<models::semantic_interface_model_t>
  get_semantic_interfaces_by_interface(const std::string& interface_name) override
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
      models::semantic_interface_model_t semantic_interface = to_semantic_interface(stmt);
      semantic_interfaces.push_back(semantic_interface);
    }

    return semantic_interfaces;
  }

  // get running model from provider
  virtual models::running_model_t get_running(const std::string& provider_name) override
  {
    // start with provider and work up the chain
    models::provider_model_t provider = get_provider(provider_name);
    if (provider.header.name.empty())
    {
      // return empty running model
      return models::running_model_t();
    }

    models::running_model_t running;
    running.interface = provider.implements;
    running.provider = provider.header.name;
    // TODO: implement this variable somehow
    running.started_by = provider_name;
    running.pid = "0";

    // get dependencies from provider depends_on
    for (auto const& [key, value] : provider.depends_on)
    {
      models::capability_model_t dependency;
      dependency.interface = key;
      dependency.provider = value;
      running.dependencies.push_back(dependency);
    }

    // return running model
    return running;
  }

  // apply a remapping to a specification model
  virtual void apply_remappings(models::specification_model_t& spec,
                                const models::remappable_base_t& remappable) override
  {
    for (auto const& param : remappable.remappings.parameters)
    {
      spec.parameters[param.from].name = param.to;
    }
    for (auto const& topic : remappable.remappings.topics)
    {
      spec.topics[topic.from].name = topic.to;
    }
    for (auto const& service : remappable.remappings.services)
    {
      spec.services[service.from].name = service.to;
    }
    for (auto const& action : remappable.remappings.actions)
    {
      spec.actions[action.from].name = action.to;
    }
  }

  // apply semantics to interface
  virtual models::interface_model_t apply_semantic_remappings(models::semantic_interface_model_t semantic) override
  {
    // get the parent interface model
    models::interface_model_t interface = get_interface(semantic.redefines);

    interface.header = semantic.header;
    apply_remappings(interface.interface, semantic);

    return interface;
  }

  // apply provider remappings
  virtual models::interface_model_t apply_provider_remappings(models::provider_model_t provider) override
  {
    // get the parent interface model
    // is it semantic
    models::semantic_interface_model_t semantic = get_semantic_interface(provider.implements);
    if (!semantic.header.name.empty())
    {
      // apply semantic remappings
      models::interface_model_t interface = apply_semantic_remappings(semantic);

      // apply provider remappings
      apply_remappings(interface.interface, provider);

      return interface;
    }

    // if not semantic, then it's an interface
    // get interface
    models::interface_model_t interface = get_interface(provider.implements);

    // apply provider remappings
    apply_remappings(interface.interface, provider);

    return interface;
  }

  // get run config model
  virtual models::run_config_model_t get_run_config(const std::string& provider_name) override
  {
    models::provider_model_t provider = get_provider(provider_name);
    if (provider.header.name.empty())
    {
      // return empty run config
      return models::run_config_model_t();
    }

    models::run_config_model_t run_config;
    run_config.provider = provider.header;
    run_config.runner = provider.runner;
    run_config.started_by = provider_name;
    run_config.pid = "0";

    // remap resources
    run_config.interface = apply_provider_remappings(provider);

    return run_config;
  }

protected:
  virtual void open() override
  {
    const int exit = sqlite3_open(db_file_.c_str(), &db_);
    if (exit != SQLITE_OK)
    {
      sqlite3_close(db_);
      throw std::runtime_error("Error opening database: " + db_file_ + " " + std::string(sqlite3_errmsg(db_)));
    }

    // enable foreign keys
    sqlite3_exec(db_, "PRAGMA foreign_keys = ON;", nullptr, nullptr, nullptr);
  }

  virtual void close() override
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

  virtual void create_tables() override
  {
    // create tables if they do not exists
    const std::string create_interface_table =
        "CREATE TABLE IF NOT EXISTS interfaces (" + models::interface_model_t::to_sql_table() + ")";

    const std::string create_semantic_interface_table =
        "CREATE TABLE IF NOT EXISTS semantic_interfaces (" + models::semantic_interface_model_t::to_sql_table() + ")";

    const std::string create_provider_table =
        "CREATE TABLE IF NOT EXISTS providers (" + models::provider_model_t::to_sql_table() + ")";

    // TODO: create meta table
    // meta table
    const std::string create_meta_table = "CREATE TABLE IF NOT EXISTS meta (key TEXT NOT NULL, value TEXT NOT NULL)";

    // execute create queries
    exec(create_interface_table);
    exec(create_semantic_interface_table);
    exec(create_provider_table);
  }

private:
  // model conversion helpers
  models::interface_model_t to_interface(sqlite3_stmt* stmt)
  {
    models::interface_model_t interface;
    interface.header.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
    interface.header.version = sqlite3_column_int(stmt, 1);
    interface.header.type = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
    interface.header.description = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));

    models::specification_model_t spec;
    spec.from_yaml(YAML::Load(std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4)))));
    interface.interface = spec;

    return interface;
  }

  models::semantic_interface_model_t to_semantic_interface(sqlite3_stmt* stmt)
  {
    models::semantic_interface_model_t semantic_interface;
    semantic_interface.header.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
    semantic_interface.header.version = sqlite3_column_int(stmt, 1);
    semantic_interface.header.type = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
    semantic_interface.header.description = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));
    semantic_interface.redefines = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4)));
    semantic_interface.global_namespace = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5)));
    semantic_interface.remappings.from_yaml(
        YAML::Load(std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6)))));

    return semantic_interface;
  }

  models::provider_model_t to_provider(sqlite3_stmt* stmt)
  {
    models::provider_model_t provider;
    provider.header.name = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0)));
    provider.header.version = sqlite3_column_int(stmt, 1);
    provider.header.type = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2)));
    provider.header.description = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 3)));
    provider.implements = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 4)));
    provider.depends_on = YAML::Load(std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 5))))
                              .as<std::map<std::string, std::string>>();
    provider.remappings.from_yaml(YAML::Load(std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 6)))));
    provider.runner = std::string(reinterpret_cast<const char*>(sqlite3_column_text(stmt, 7)));

    return provider;
  }

private:
  const std::string db_file_;
  sqlite3* db_;
};

}  // namespace capabilities2_server
