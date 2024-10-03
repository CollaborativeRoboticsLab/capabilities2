#include <tinyxml2.h>
#include <string>
#include <vector>
#include <capabilities2_executor/structs/connection.hpp>

namespace capabilities2_xml_parser
{
	/**
	 * @brief extract elements related plan
	 *
	 * @param document XML document to extract plan from
	 *
	 * @return plan in the form of tinyxml2::XMLElement*
	 */
	tinyxml2::XMLElement *get_plan(tinyxml2::XMLDocument &document)
	{
		return document.FirstChildElement("Plan");
	}

	/**
	 * @brief search a string in a vector of strings
	 *
	 * @param list vector of strings to be searched
	 * @param value string to be searched in the vector
	 *
	 * @return `true` if value is found in list and `false` otherwise
	 */
	bool search(std::vector<std::string> list, std::string value)
	{
		return (std::find(list.begin(), list.end(), value) != list.end());
	}

	/**
	 * @brief check if the element in question is the last in its level
	 *
	 * @param element element to be checked
	 *
	 * @return `true` if last and `false` otherwise
	 */
	bool isLastElement(tinyxml2::XMLElement *element)
	{
		return (element == element->Parent()->LastChildElement());
	}

	/**
	 * @brief check if the xml document has valid plan tags
	 *
	 * @param document XML document in question
	 *
	 * @return `true` if its valid and `false` otherwise
	 */
	bool check_plan_tag(tinyxml2::XMLDocument &document)
	{
		std::string plan_tag(document.FirstChildElement()->Name());

		if (plan_tag == "Plan")
			return true;
		else
			return false;
	}

	/**
	 * @brief convert XMLElement to std::string
	 *
	 * @param element element to be converted
	 *
	 * @return std::string converted element
	 */
	std::string convert_to_string(tinyxml2::XMLElement *element)
	{
		tinyxml2::XMLPrinter printer;
		
		element->Accept(&printer);

		std::string value = printer.CStr();

		return value;
	}

	/**
	 * @brief convert XMLDocument to std::string
	 *
	 * @param document element to be converted
	 *
	 * @return std::string converted document
	 */
	std::string convert_to_string(tinyxml2::XMLDocument &document)
	{
		tinyxml2::XMLPrinter printer;
		
		document.Accept(&printer);

		std::string value = printer.CStr();

		return value;
	}

	/**
	 * @brief check the plan for invalid/unsupported control and event tags
	 * uses recursive approach to go through the plan
	 *
	 * @param element XML Element to be evaluated
	 * @param events_list list containing valid event tags
	 * @param providers_list list containing providers
	 * @param control_list list containing valid control tags
	 *
	 * @return `true` if element valid and supported and `false` otherwise
	 */
	bool check_tags(tinyxml2::XMLElement *element,
					std::vector<std::string> &events_list,
					std::vector<std::string> &providers_list,
					std::vector<std::string> &control_list)
	{
		const char **name;
		const char **provider;

		element->QueryStringAttribute("name", name);
		element->QueryStringAttribute("provider", provider);

		std::string typetag(element->Name());
		std::string nametag(*name);
		std::string providertag(*provider);

		bool hasChildren = !element->NoChildren();
		bool hasSiblings = !capabilities2_xml_parser::isLastElement(element);
		bool foundInControl = capabilities2_xml_parser::search(control_list, nametag);
		bool foundInEvents = capabilities2_xml_parser::search(events_list, nametag);
		bool foundInProviders = capabilities2_xml_parser::search(providers_list, providertag);
		bool returnValue = true;

		if (typetag == "Control")
		{
			if (foundInControl and hasChildren)
				returnValue = returnValue and capabilities2_xml_parser::check_tags(element->FirstChildElement(), events_list, providers_list, control_list);

			if (foundInControl and hasSiblings)
				returnValue = returnValue and capabilities2_xml_parser::check_tags(element->NextSiblingElement(), events_list, providers_list, control_list);

			if (foundInControl and !hasSiblings)
				returnValue = returnValue;

			if (!foundInControl)
				return false;
		}
		else if (typetag == "Event")
		{
			if (foundInEvents and foundInProviders and hasSiblings)
				returnValue = returnValue and capabilities2_xml_parser::check_tags(element->NextSiblingElement(), events_list, providers_list, control_list);

			if (foundInEvents and foundInProviders and !hasSiblings)
				returnValue = returnValue;

			if (!foundInEvents)
				return false;

			if (!foundInProviders)
				return false;
		}
		else
		{
			return false;
		}

		return returnValue;
	}

	/**
	 * @brief Adds an event connection to the capabilities2 fabric
	 *
	 * @param source_event the source event from which the connection would start
	 * @param source_provider provider of the source event
	 * @param target_event the target event which will be connected via the connection
	 * @param target_provider provider of the target event
	 * @param connection the type of connection
	 * @param event_element the tinyxml2::XMLElement which contains runtime parameters
	 *
	 * @return The connection object
	 */
	capabilities2_executor::connection_t create_connection(std::string source_event, std::string source_provider,
														   std::string target_event, std::string target_provider,
														   capabilities2_executor::connection_type_t connection, tinyxml2::XMLElement *event_element)
	{
		capabilities2_executor::connection_t connect;

		connect.source_event = source_event;
		connect.source_provider = source_provider;
		connect.target_event = target_event;
		connect.target_provider = target_provider;
		connect.connection = connection;
		connect.event_element = event_element;

		return connect;
	}

	/**
	 * @brief parse through the plan and extract the connections
	 *
	 * @param element XML Element to be evaluated
	 * @param connection_list std::vector containing extracted connections
	 * @param connection the type of connection
	 * @param source_event source event name. if not provided, "start" is taken as default
	 * @param source_provider provider of the source event, if not provided "" is taken as default
	 * @param target_event target event name. if not provided, "stop" is taken as default
	 * @param target_provider provider of the target event. if not provided, "stop" is taken as default
	 */
	void extract_connections(tinyxml2::XMLElement *element, std::vector<capabilities2_executor::connection_t> &connection_list,
							 capabilities2_executor::connection_type_t connection = capabilities2_executor::connection_type_t::ON_SUCCESS_START,
							 std::string source_event = "start", std::string source_provider = "",
							 std::string target_event = "stop", std::string target_provider = "")
	{
		const char **name;
		const char **provider;

		element->QueryStringAttribute("name", name);
		element->QueryStringAttribute("provider", provider);

		std::string typetag(element->Name());
		std::string nametag(*name);
		std::string providertag(*provider);

		bool hasChildren = !element->NoChildren();
		bool hasSiblings = !capabilities2_xml_parser::isLastElement(element);

		if ((typetag == "Control") and (nametag == "sequential"))
		{
			if (hasChildren)
				capabilities2_xml_parser::extract_connections(element->FirstChildElement(), connection_list, capabilities2_executor::connection_type_t::ON_SUCCESS_START,
															  source_event, source_provider, target_event, target_provider);

			if (hasSiblings)
				capabilities2_xml_parser::extract_connections(element->NextSiblingElement(), connection_list, connection,
															  source_event, source_provider, target_event, target_provider);
		}
		else if ((typetag == "Control") and (nametag == "parallel"))
		{
			if (hasChildren)
				capabilities2_xml_parser::extract_connections(element->FirstChildElement(), connection_list, capabilities2_executor::connection_type_t::ON_START_START,
															  source_event, source_provider, target_event, target_provider);

			if (hasSiblings)
				capabilities2_xml_parser::extract_connections(element->NextSiblingElement(), connection_list, connection,
															  source_event, source_provider, target_event, target_provider);
		}
		else if ((typetag == "Control") and (nametag == "recovery"))
		{
			if (hasChildren)
				capabilities2_xml_parser::extract_connections(element->FirstChildElement(), connection_list, capabilities2_executor::connection_type_t::ON_FAILURE_START,
															  source_event, source_provider, target_event, target_provider);

			if (hasSiblings)
				capabilities2_xml_parser::extract_connections(element->NextSiblingElement(), connection_list, connection,
															  source_event, source_provider, target_event, target_provider);
		}
		else if (typetag == "Event")
		{
			capabilities2_executor::connection_t connection_obj = create_connection(source_event, source_provider, nametag, providertag, connection, element);
			connection_list.push_back(connection_obj);

			if (hasSiblings)
				capabilities2_xml_parser::extract_connections(element->NextSiblingElement(), connection_list, connection,
															  nametag, providertag, target_event, target_provider);
		}
	}

} // namespace capabilities2_xml_parser
