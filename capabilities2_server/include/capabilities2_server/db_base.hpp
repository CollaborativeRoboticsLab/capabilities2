#pragma once

#include <string>
#include <capabilities2_server/models/interface.hpp>
#include <capabilities2_server/models/semantic_interface.hpp>
#include <capabilities2_server/models/provider.hpp>
#include <capabilities2_server/models/remappable_base.hpp>
#include <capabilities2_server/models/running.hpp>
#include <capabilities2_server/models/run_config.hpp>

namespace capabilities2_server
{
class DBBase
{
public:
  DBBase() = default;
  virtual ~DBBase() = default;

  /* inserters */

  // insert interface
  virtual void insert_interface(const models::interface_model_t& interface) = 0;

  // insert semantic interface
  virtual void insert_semantic_interface(const models::semantic_interface_model_t& semantic_interface) = 0;

  // insert provider
  virtual void insert_provider(const models::provider_model_t& provider) = 0;

  /* getters */

  // get interface
  virtual models::interface_model_t get_interface(const std::string& name) = 0;

  // get interfaces
  virtual std::vector<models::interface_model_t> get_interfaces() = 0;

  // get semantic interface
  virtual models::semantic_interface_model_t get_semantic_interface(const std::string& name) = 0;

  // get semantic interfaces
  virtual std::vector<models::semantic_interface_model_t> get_semantic_interfaces() = 0;

  // get provider
  virtual models::provider_model_t get_provider(const std::string& name) = 0;

  // get providers
  virtual std::vector<models::provider_model_t> get_providers() = 0;

  /**
   * @brief Get remappable model by name
   *
   * remappable models are either semantic interfaces or providers
   *
   * @param name
   * @return models::remappable_base_t
   */
  virtual models::remappable_base_t get_remappable(const std::string& name) = 0;

  /* updaters */

  // update interface
  virtual void update_interface(const models::interface_model_t& interface) = 0;

  // update semantic interface
  virtual void update_semantic_interface(const models::semantic_interface_model_t& semantic_interface) = 0;

  // update provider
  virtual void update_provider(const models::provider_model_t& provider) = 0;

  /* foreign keys */

  // get providers by interface
  virtual std::vector<models::provider_model_t> get_providers_by_interface(const std::string& interface_name) = 0;

  // get semantic interfaces by interface
  virtual std::vector<models::semantic_interface_model_t>
  get_semantic_interfaces_by_interface(const std::string& interface_name) = 0;

  // get running model from provider
  virtual models::running_model_t get_running(const std::string& provider_name) = 0;

  // apply a remapping to a specification model
  virtual void apply_remappings(models::specification_model_t& spec, const models::remappable_base_t& remappable) = 0;

  // apply semantics to interface
  virtual models::interface_model_t apply_semantic_remappings(models::semantic_interface_model_t semantic) = 0;

  // apply provider remappings
  virtual models::interface_model_t apply_provider_remappings(models::provider_model_t provider) = 0;

  // get run config model
  virtual models::run_config_model_t get_run_config(const std::string& provider_name) = 0;

  // exists in db templated by model type
  template <typename T>
  bool exists(const std::string& resource_name)
  {
    // clearly not in db as it does not match a valid model type
    throw std::runtime_error("invalid model type");
  }

  // exists in db anywhere
  virtual bool exists_any(const std::string& resource_name)
  {
    // query all tables for name
    // if name is in any table, then it exists
    return !get_interface(resource_name).header.name.empty() ||
           !get_semantic_interface(resource_name).header.name.empty() ||
           !get_provider(resource_name).header.name.empty();
  }

protected:
  virtual void open() = 0;
  virtual void close() = 0;
  virtual void create_tables() = 0;
};

// 'exists' template specializations
// interface exists in db
template <>
bool DBBase::exists<models::interface_model_t>(const std::string& resource_name)
{
  return !get_interface(resource_name).header.name.empty();
}

// semantic interface exists in db
template <>
bool DBBase::exists<models::semantic_interface_model_t>(const std::string& resource_name)
{
  return !get_semantic_interface(resource_name).header.name.empty();
}

// provider exists in db
template <>
bool DBBase::exists<models::provider_model_t>(const std::string& resource_name)
{
  return !get_provider(resource_name).header.name.empty();
}

}  // namespace capabilities2_server
