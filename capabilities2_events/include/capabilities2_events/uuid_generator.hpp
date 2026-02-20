#pragma once

#include <string>

namespace capabilities2_events
{
class UUIDGenerator
{
public:
  /**
   * @brief generate a uuid string
   *
   * @return const std::string
   */
  static const std::string gen_uuid_str();

private:
  // delete constructor and assignment operator to prevent instantiation
  UUIDGenerator() = delete;
  UUIDGenerator(const UUIDGenerator&) = delete;
  UUIDGenerator& operator=(const UUIDGenerator&) = delete;
};
}  // namespace capabilities2_events
