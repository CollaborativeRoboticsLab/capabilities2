#include <string>
#include <tinyxml2.h>

namespace capabilities2_executor
{
	enum connection_type_t
	{
		ON_START,
		ON_SUCCESS,
		ON_FAILURE,
		ON_STOP
	};

	struct connection_t
	{
		std::string runner;
		std::string provider;
		tinyxml2::XMLElement* parameters = nullptr;
	};

	struct node_t {
		connection_t source;
		connection_t target_on_start;
		connection_t target_on_stop;
		connection_t target_on_success;
		connection_t target_on_failure;
	};

} // namespace capabilities2_executor
