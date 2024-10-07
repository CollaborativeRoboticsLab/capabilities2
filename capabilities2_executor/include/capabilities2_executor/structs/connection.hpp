#include <string>
#include <tinyxml2.h>

namespace capabilities2_executor
{
	enum connection_type_t
	{
		ON_START_START,
		ON_START_STOP,
		ON_SUCCESS_START,
		ON_SUCCESS_STOP,
		ON_FAILURE_START,
		ON_FAILURE_STOP,
		ON_TERMINATE_START,
		ON_TERMINATE_STOP
	};

	struct connection_t {
		std::string source_event;
		std::string source_provider;
		std::string target_event;
		std::string target_provider;
		connection_type_t connection;
		tinyxml2::XMLElement* event_element;
	};

} // namespace capabilities2_executor
