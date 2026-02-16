#pragma once

#include <string>
#include <vector>
#include <any>
#include <stdexcept>
#include <capabilities2_msgs/msg/capability.hpp>
#include <capabilities2_msgs/msg/capability_parameter.hpp>

namespace capabilities2_events
{

struct options_exception : public std::runtime_error
{
  using std::runtime_error::runtime_error;

  options_exception(const std::string& what) : std::runtime_error(what)
  {
  }

  virtual const char* what() const noexcept override
  {
    return std::runtime_error::what();
  }
};

enum class OptionType
{
  BOOL,
  DOUBLE,
  INT,
  STRING,
  UNCONVERTED,
  VECTOR_BOOL,
  VECTOR_DOUBLE,
  VECTOR_INT,
  VECTOR_STRING
};

/**
 * @brief Key value pair for capability options
 *
 * @param key the key of the option
 * @param value the value of the option
 */
struct Parameter
{
  std::string key;
  std::vector<std::string> value;
  OptionType type;

  Parameter() = default;

  Parameter(const capabilities2_msgs::msg::CapabilityParameter& msg)
  {
    key = msg.key;
    value = msg.value;
    type = static_cast<OptionType>(msg.type);
  }

  std::any get_value()
  {
    switch (type)
    {
      case OptionType::BOOL:
        return value[0] == "true";
      case OptionType::DOUBLE:
        return std::stod(value[0]);
      case OptionType::INT:
        return std::stoi(value[0]);
      case OptionType::STRING:
        return value[0];
      case OptionType::VECTOR_BOOL: {
        std::vector<bool> vec;
        for (const auto& v : value)
          vec.push_back(v == "true");
        return vec;
      }
      case OptionType::VECTOR_DOUBLE: {
        std::vector<double> vec;
        for (const auto& v : value)
          vec.push_back(std::stod(v));
        return vec;
      }
      case OptionType::VECTOR_INT: {
        std::vector<int> vec;
        for (const auto& v : value)
          vec.push_back(std::stoi(v));
        return vec;
      }
      case OptionType::VECTOR_STRING:
        return value;
    }
  }

  void set_value(std::string new_key, std::any new_value, OptionType new_type)
  {
    key = new_key;
    type = new_type;

    switch (type)
    {
      case OptionType::BOOL:
        value.clear();
        value.push_back(std::any_cast<bool>(new_value) ? "true" : "false");
        return;
      case OptionType::DOUBLE:
        value.clear();
        value.push_back(std::to_string(std::any_cast<double>(new_value)));
        return;
      case OptionType::INT:
        value.clear();
        value.push_back(std::to_string(std::any_cast<int>(new_value)));
        return;
      case OptionType::STRING:
        value.clear();
        value.push_back(std::any_cast<std::string>(new_value));
        return;
      case OptionType::VECTOR_BOOL: {
        const auto& vec = std::any_cast<std::vector<bool>>(new_value);
        value.clear();
        for (const auto& v : vec)
          value.push_back(v ? "true" : "false");
        return;
      }
      case OptionType::VECTOR_DOUBLE: {
        const auto& vec = std::any_cast<std::vector<double>>(new_value);
        value.clear();
        for (const auto& v : vec)
          value.push_back(std::to_string(v));
        return;
      }
      case OptionType::VECTOR_INT: {
        const auto& vec = std::any_cast<std::vector<int>>(new_value);
        value.clear();
        for (const auto& v : vec)
          value.push_back(std::to_string(v));
        return;
      }
      case OptionType::VECTOR_STRING:
        value = std::any_cast<std::vector<std::string>>(new_value);
        return;

      default:
        throw options_exception("Unsupported OptionType");
    }
  }

  capabilities2_msgs::msg::CapabilityParameter toMsg() const
  {
    capabilities2_msgs::msg::CapabilityParameter msg;
    msg.key = key;
    msg.value = value;
    msg.type = static_cast<int>(type);
    return msg;
  }
};

/**
 * @brief capability options for a capability runner given by interface and provider
 *
 * @param interface the interface of the capability
 * @param provider the provider of the capability
 * @param options the options for the capability
 */
struct EventParameters
{
  std::vector<Parameter> options = {};

  EventParameters() = default;

  EventParameters(const capabilities2_msgs::msg::Capability& msg)
  {
    options.clear();
    for (const auto& option_msg : msg.parameters)
    {
      auto option = Parameter(option_msg);
      options.push_back(option);
    }
  }

  bool is_empty() const
  {
    return options.empty();
  }

  /**
   * @brief Check if an option with the given key exists
   *
   * @param key the key of the option to check
   * @return true if the option exists, false otherwise
   */
  bool has_value(const std::string& key) const
  {
    for (const auto& option : options)
      if (option.key == key)
        return true;
    return false;
  }

  /**
   * @brief Get the value of an option by key
   *
   * @param key the key of the option to get the value of
   * @return std::any the value of the option, can be cast to the appropriate type based on the OptionType
   * @throws options_exception if the key is not found or if there is a type conversion error
   */
  std::any get_value(const std::string& key, std::any default_value)
  {
    if (has_value(key))
      for (auto& option : options)
      {
        if (option.key == key)
        {
          try
          {
            return option.get_value();
          }
          catch (const std::exception& e)
          {
            throw options_exception("Failed to convert option '" + option.key + "': " + e.what());
          }
        }
      }
    else
      return default_value;
  }

  /**
   * @brief Set the value of an option by key, if the option does not exist it will be created
   *
   * @param key the key of the option to set the value of
   * @param type the type of the option to set
   * @param value the value to set, should be castable to the appropriate type based on the OptionType
   * @throws options_exception if there is a type conversion error
   */
  void set_value(const std::string& key, const std::any& value, const OptionType& type)
  {
    for (auto& option : options)
      if (option.key == key)
      {
        try
        {
          option.set_value(key, value, type);
          return;
        }
        catch (const std::exception& e)
        {
          throw options_exception("Failed to set option '" + option.key + "': " + e.what());
        }
      }

    // Parameter not found, add new one
    Parameter new_option;
    new_option.set_value(key, value, type);
    options.push_back(new_option);
  }

  capabilities2_msgs::msg::Capability toMsg() const
  {
    capabilities2_msgs::msg::Capability msg;
    for (const auto& option : options)
      msg.parameters.push_back(option.toMsg());
    return msg;
  }
};

}  // namespace capabilities2_events