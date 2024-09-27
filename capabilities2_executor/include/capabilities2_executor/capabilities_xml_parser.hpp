#include <tinyxml2.h>
#include <string>
#include <vector>

namespace capabilities2_xml_parser
{
	/**
	 * @brief extract elements related plan
	 *
	 * @param document XML document to extract plan from
	 *
	 * @return plan in the form of tinyxml2::XMLElement*
	 */
	tinyxml2::XMLElement* get_plan(tinyxml2::XMLDocument &document)
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
	 * @brief check the plan for invalid/unsupported control and event tags
	 * uses recursive approach to go through the plan
	 *
	 * @param element XML Element to be evaluated
	 * @param events_list std::string containing valid event tags
	 * @param control_list std::string containing valid control tags
	 *
	 * @return `true` if element valid and supported and `false` otherwise
	 */
	bool check_tags(tinyxml2::XMLElement *element,
					std::vector<std::string> &events_list,
					std::vector<std::string> &control_list)
	{
		const char **name;

		element->QueryStringAttribute("name", name);

		std::string typetag(element->Name());
		std::string nametag(*name);

		bool hasChildren 	= !element->NoChildren();
		bool hasSiblings 	= !capabilities2_xml_parser::isLastElement(element);
		bool foundInControl = capabilities2_xml_parser::search(control_list, nametag);
		bool foundInEvents  = capabilities2_xml_parser::search(events_list, nametag);
		bool returnValue 	= true;

		if (typetag == "Control")
		{
			if (foundInControl and hasChildren)
				returnValue = returnValue and capabilities2_xml_parser::check_tags(element->FirstChildElement(), events_list, control_list);

			if (foundInControl and hasSiblings)
				returnValue = returnValue and capabilities2_xml_parser::check_tags(element->NextSiblingElement(), events_list, control_list);

			if (foundInControl and !hasSiblings)
				returnValue = returnValue;

			if (!foundInControl)
				return false;
		}
		else if (typetag == "Event")
		{
			if (foundInEvents and hasSiblings)
				returnValue = returnValue and capabilities2_xml_parser::check_tags(element->NextSiblingElement(), events_list, control_list);

			if (foundInEvents and !hasSiblings)
				returnValue = returnValue;

			if (!foundInEvents)
				return false;
		}
		else
		{
			return false;
		}

		return returnValue;
	}

	

} // namespace capabilities2_xml_parser
