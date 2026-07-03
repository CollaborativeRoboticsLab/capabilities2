#pragma once
#include <string>
#include <regex>

namespace capabilities2_server
{

/**
 * @brief convert a generic std::string to a sql safe std::string 
 *
 * @param input generic std::string
 * @return sql safe std::string
 *
 */
std::string to_sql_safe(const std::string& input)
{
  std::string result = input;

  // Escape backslashes (if necessary for your SQL dialect)
  result = std::regex_replace(result, std::regex(R"(\\)"), R"(\\\\)");

  // Escape single quotes by replacing them with two single quotes
  result = std::regex_replace(result, std::regex(R"(')"), R"('')");

  return result;
}

/**
 * @brief convert a sql safe std::string to a generic std::string 
 *
 * @param input sql safe std::string
 * @return generic std::string
 *
 */
std::string from_sql_safe(const std::string& input)
{
  std::string result = input;

  // Escape backslashes (if necessary for your SQL dialect)
  result = std::regex_replace(result, std::regex(R"(\\\\)"), R"(\\)");

  // Escape single quotes by replacing them with two single quotes
  result = std::regex_replace(result, std::regex(R"('')"), R"(')");

  return result;
}
}  // namespace capabilities2_server
