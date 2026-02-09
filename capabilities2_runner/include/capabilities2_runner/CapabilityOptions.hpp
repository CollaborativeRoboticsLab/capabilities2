#pragma once

#include <string>
#include <vector>
#include <any>
#include <stdexcept>
#include <capabilities2_runner/utils/exceptions.hpp>
#include <capabilities2_msgs/msg/capability_option.hpp>
#include <capabilities2_msgs/msg/capability.hpp>

namespace capabilities2
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
struct Options
{
  std::string key;
  std::vector<std::string> value;
  OptionType type;

  capabilities2_msgs::msg::CapabilityOption toMsg() const
  {
    capabilities2_msgs::msg::CapabilityOption msg;
    msg.key = key;
    msg.value = value;
    msg.type = static_cast<int>(type);
    return msg;
  }

  void fromMsg(const capabilities2_msgs::msg::CapabilityOption& msg)
  {
    key = msg.key;
    value = msg.value;
    type = static_cast<OptionType>(msg.type);
  }
};

/**
 * @brief capability options for a capability runner given by interface and provider
 *
 * @param interface the interface of the capability
 * @param provider the provider of the capability
 * @param options the options for the capability
 */
struct CapabilityOptions
{
  std::string interface;
  std::string provider;
  std::vector<Options> options;

  std::any get_value(const std::string& key) const
  {
    for (const auto& option : options)
      if (option.key == key)
      {
        try
        {
          switch (option.type)
          {
            case OptionType::BOOL:
              return option.value[0] == "true";
            case OptionType::DOUBLE:
              return std::stod(option.value[0]);
            case OptionType::INT:
              return std::stoi(option.value[0]);
            case OptionType::STRING:
              return option.value[0];
            case OptionType::VECTOR_BOOL: {
              std::vector<bool> vec;
              for (const auto& v : option.value)
                vec.push_back(v == "true");
              return vec;
            }
            case OptionType::VECTOR_DOUBLE: {
              std::vector<double> vec;
              for (const auto& v : option.value)
                vec.push_back(std::stod(v));
              return vec;
            }
            case OptionType::VECTOR_INT: {
              std::vector<int> vec;
              for (const auto& v : option.value)
                vec.push_back(std::stoi(v));
              return vec;
            }
            case OptionType::VECTOR_STRING:
              return option.value;
          }
        }
        catch (const std::exception& e)
        {
          throw options_exception("Failed to convert option '" + option.key + "': " + e.what());
        }

        throw options_exception("Unknown option type for key: " + option.key);
      }
  }

  void set_value(const std::string& key, const OptionType& type, const std::any& value)
  {
    for (auto& option : options)
      if (option.key == key)
      {
        switch (type)
        {
          case OptionType::BOOL:
            option.value[0] = std::any_cast<bool>(value) ? "true" : "false";
            return;
          case OptionType::DOUBLE:
            option.value[0] = std::to_string(std::any_cast<double>(value));
            return;
          case OptionType::INT:
            option.value[0] = std::to_string(std::any_cast<int>(value));
            return;
          case OptionType::STRING:
            option.value[0] = std::any_cast<std::string>(value);
            return;
          case OptionType::VECTOR_BOOL: {
            const auto& vec = std::any_cast<std::vector<bool>>(value);
            option.value.clear();
            for (const auto& v : vec)
              option.value.push_back(v ? "true" : "false");
            return;
          }
          case OptionType::VECTOR_DOUBLE: {
            const auto& vec = std::any_cast<std::vector<double>>(value);
            option.value.clear();
            for (const auto& v : vec)
              option.value.push_back(std::to_string(v));
            return;
          }
          case OptionType::VECTOR_INT: {
            const auto& vec = std::any_cast<std::vector<int>>(value);
            option.value.clear();
            for (const auto& v : vec)
              option.value.push_back(std::to_string(v));
            return;
          }
          case OptionType::VECTOR_STRING:
            option.value = std::any_cast<std::vector<std::string>>(value);
            return;
        }
      }

    // Option not found, add new one
    Options new_option;
    new_option.key = key;
    new_option.type = type;

    switch (type)
    {
      case OptionType::BOOL:
        new_option.value[0] = std::any_cast<bool>(value) ? "true" : "false";
        break;
      case OptionType::DOUBLE:
        new_option.value[0] = std::to_string(std::any_cast<double>(value));
        break;
      case OptionType::INT:
        new_option.value[0] = std::to_string(std::any_cast<int>(value));
        break;
      case OptionType::STRING:
        new_option.value[0] = std::any_cast<std::string>(value);
        break;
      case OptionType::VECTOR_BOOL: {
        const auto& vec = std::any_cast<std::vector<bool>>(value);
        for (const auto& v : vec)
          new_option.value.push_back(v ? "true" : "false");
        break;
      }
      case OptionType::VECTOR_DOUBLE: {
        const auto& vec = std::any_cast<std::vector<double>>(value);
        for (const auto& v : vec)
          new_option.value.push_back(std::to_string(v));
        break;
      }
      case OptionType::VECTOR_INT: {
        const auto& vec = std::any_cast<std::vector<int>>(value);
        for (const auto& v : vec)
          new_option.value.push_back(std::to_string(v));
        break;
      }
      case OptionType::VECTOR_STRING:
        new_option.value = std::any_cast<std::vector<std::string>>(value);
        break;
    };

    options.push_back(new_option);
  }

  capabilities2_msgs::msg::Capability toMsg() const
  {
    capabilities2_msgs::msg::Capability msg;
    msg.interface = interface;
    msg.provider = provider;
    for (const auto& option : options)
      msg.options.push_back(option.toMsg());
    return msg;
  }

  void fromMsg(const capabilities2_msgs::msg::Capability& msg)
  {
    interface = msg.interface;
    provider = msg.provider;
    options.clear();
    for (const auto& option_msg : msg.options)
    {
      Options option;
      option.fromMsg(option_msg);
      options.push_back(option);
    }
  }
};

}  // namespace capabilities2