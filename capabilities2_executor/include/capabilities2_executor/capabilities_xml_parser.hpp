#include <tinyxml2.h>
#include <string>
#include <vector>

namespace capabilities2_xml_parser
{
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

	bool check_plan_tag(tinyxml2::XMLDocument &document)
	{
		std::string plan_tag(document.FirstChildElement()->Name());

		if (plan_tag == "Plan")
			return true;
		else
			return false;
	}

	bool check_tags(tinyxml2::XMLElement *element,
					std::vector<std::string> &actions_list,
					std::vector<std::string> &control_list)
	{
		const char **name;

		element->QueryStringAttribute("name", name);

		std::string typetag(element->Name());
		std::string nametag(*name);

		bool hasChildren = !element->NoChildren();
		bool hasSiblings = !capabilities2_xml_parser::isLastElement(element);
		bool foundInControl = capabilities2_xml_parser::search(control_list, nametag);
		bool foundInActions = capabilities2_xml_parser::search(actions_list, nametag);
		bool returnValue = true;

		if (typetag == "Control")
		{
			if (foundInControl and hasChildren)
				returnValue = returnValue and capabilities2_xml_parser::check_tags(element->FirstChildElement(), actions_list, control_list);

			if (foundInControl and hasSiblings)
				returnValue = returnValue and capabilities2_xml_parser::check_tags(element->NextSiblingElement(), actions_list, control_list);

			if (foundInControl and !hasSiblings)
				returnValue = returnValue;

			if (!foundInControl)
				return false;
		}
		else if (typetag == "Event")
		{
			if (foundInActions and hasSiblings)
				returnValue = returnValue and capabilities2_xml_parser::check_tags(element->NextSiblingElement(), actions_list, control_list);

			if (foundInActions and !hasSiblings)
				returnValue = returnValue;

			if (!foundInActions)
				return false;
		}
		else
		{
			return false;
		}

		return returnValue;
	}

	tinyxml2::XMLElement *get_plan(tinyxml2::XMLDocument &document)
	{
		return document.FirstChildElement("Plan");
	}

} // namespace capabilities2_xml_parser
