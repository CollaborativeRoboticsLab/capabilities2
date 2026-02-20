#include <capabilities2_events/uuid_generator.hpp>
#include <uuid/uuid.h>

namespace capabilities2_events
{
/**
 * @brief generate a uuid string
 *
 * @return const std::string
 */
const std::string UUIDGenerator::gen_uuid_str()
{
  // generate a new uuid
  uuid_t uuid;
  uuid_generate_random(uuid);
  char uuid_str[40];
  uuid_unparse(uuid, uuid_str);
  return std::string(uuid_str);
}
}  // namespace capabilities2_events
