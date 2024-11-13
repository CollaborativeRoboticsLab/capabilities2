#include <tinyxml2.h>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
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
tinyxml2::XMLElement* get_plan(tinyxml2::XMLDocument& document)
{
  return document.FirstChildElement("Plan")->FirstChildElement();
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
bool isLastElement(tinyxml2::XMLElement* element)
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
bool check_plan_tag(tinyxml2::XMLDocument& document)
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
 * @param element XMLElement element to be converted
 * @param paramters parameter to hold std::string
 *
 * @return `true` if element is not nullptr and conversion successful, `false` if element is nullptr
 */
bool convert_to_string(tinyxml2::XMLElement* element, std::string& paramters)
{
  if (element)
  {
    tinyxml2::XMLPrinter printer;
    element->Accept(&printer);
    paramters = printer.CStr();
    return true;
  }
  else
  {
    paramters = "";
    return false;
  }
}

/**
 * @brief convert XMLDocument to std::string
 *
 * @param document element to be converted
 *
 * @return std::string converted document
 */
void convert_to_string(tinyxml2::XMLDocument& document_xml, std::string& document_string)
{
  tinyxml2::XMLPrinter printer;
  document_xml.Print(&printer);
  document_string = printer.CStr();
}

/**
 * @brief check the plan for invalid/unsupported control and event tags
 * uses recursive approach to go through the plan
 *
 * @param element XML Element to be evaluated
 * @param events list containing valid event tags
 * @param providers list containing providers
 * @param control list containing valid control tags
 * @param rejected list containing invalid tags
 *
 * @return `true` if element valid and supported and `false` otherwise
 */
bool check_tags(rclcpp::Logger logger, tinyxml2::XMLElement* element, std::vector<std::string>& events, std::vector<std::string>& providers,
                std::vector<std::string>& control, std::vector<std::string>& rejected)
{
  const char* name;
  const char* provider;

  std::string parameter_string;
  convert_to_string(element, parameter_string);

  element->QueryStringAttribute("name", &name);
  element->QueryStringAttribute("provider", &provider);

  std::string nametag;
  std::string providertag;

  // // Check and retrieve "name" and "provider" attributes safely
  // if (element->QueryStringAttribute("name", &name) != tinyxml2::XML_SUCCESS || name == nullptr)
  // {
  //   RCLCPP_ERROR(logger, "Missing 'name' attribute in XML element : %s", parameter_string.c_str());
  //   rejected.push_back(parameter_string);
  //   return false;
  // }

  // if (element->QueryStringAttribute("provider", &provider) != tinyxml2::XML_SUCCESS || provider == nullptr)
  // {
  //   RCLCPP_ERROR(logger, "Missing 'provider' attribute in XML element : %s", parameter_string.c_str());
  //   rejected.push_back(parameter_string);
  //   return false;
  // }

  std::string typetag(element->Name());

  if (name)
    nametag = name;
  else
    nametag = "";

  if (provider)
    providertag = provider;
  else
    providertag = "";

  bool hasChildren = !element->NoChildren();
  bool hasSiblings = !capabilities2_xml_parser::isLastElement(element);
  bool foundInControl = capabilities2_xml_parser::search(control, nametag);
  bool foundInEvents = capabilities2_xml_parser::search(events, nametag);
  bool foundInProviders = capabilities2_xml_parser::search(providers, providertag);
  bool returnValue = true;

  if (typetag == "Control")
  {
    if (!foundInControl)
    {
      RCLCPP_ERROR(logger, "Control tag '%s' not available in the valid list", nametag.c_str());
      rejected.push_back(parameter_string);
      return false;
    }

    if (hasChildren)
      returnValue &= capabilities2_xml_parser::check_tags(logger, element->FirstChildElement(), events, providers, control, rejected);

    if (hasSiblings)
      returnValue &= capabilities2_xml_parser::check_tags(logger, element->NextSiblingElement(), events, providers, control, rejected);
  }
  else if (typetag == "Event")
  {
    if (!foundInEvents || !foundInProviders)
    {
      RCLCPP_ERROR(logger, "Event tag name '%s' or provider '%s' not available in the valid list", nametag.c_str(), providertag.c_str());
      rejected.push_back(parameter_string);
      return false;
    }

    if (hasSiblings)
      returnValue &= capabilities2_xml_parser::check_tags(logger, element->NextSiblingElement(), events, providers, control, rejected);
  }
  else
  {
    RCLCPP_ERROR(logger, "xml element is not valid", parameter_string.c_str());
    rejected.push_back(parameter_string);
    return false;
  }

  return returnValue;
}

/**
 * @brief Returns control xml tags supported in extract_connections method
 *
 */
std::vector<std::string> get_control_list()
{
  std::vector<std::string> tag_list;

  tag_list.push_back("sequential");
  tag_list.push_back("parallel");
  tag_list.push_back("recovery");

  return tag_list;
}

/**
 * @brief parse through the plan and extract the connections
 *
 * @param element XML Element to be evaluated
 * @param connections std::map containing extracted connections
 * @param connection_id numerical id of the connection
 * @param connection_type the type of connection
 */
void extract_connections(tinyxml2::XMLElement* element, std::map<int, capabilities2_executor::node_t>& connections, int connection_id = 0,
                         capabilities2_executor::connection_type_t connection_type = capabilities2_executor::connection_type_t::ON_SUCCESS)
{
  int predecessor_id;

  const char** name;
  const char** provider;

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
      capabilities2_xml_parser::extract_connections(element->FirstChildElement(), connections, connection_id,
                                                    capabilities2_executor::connection_type_t::ON_SUCCESS);

    if (hasSiblings)
      capabilities2_xml_parser::extract_connections(element->NextSiblingElement(), connections, connection_id, connection_type);
  }
  else if ((typetag == "Control") and (nametag == "parallel"))
  {
    if (hasChildren)
      capabilities2_xml_parser::extract_connections(element->FirstChildElement(), connections, connection_id,
                                                    capabilities2_executor::connection_type_t::ON_START);

    if (hasSiblings)
      capabilities2_xml_parser::extract_connections(element->NextSiblingElement(), connections, connection_id, connection_type);
  }
  else if ((typetag == "Control") and (nametag == "recovery"))
  {
    if (hasChildren)
      capabilities2_xml_parser::extract_connections(element->FirstChildElement(), connections, connection_id,
                                                    capabilities2_executor::connection_type_t::ON_FAILURE);

    if (hasSiblings)
      capabilities2_xml_parser::extract_connections(element->NextSiblingElement(), connections, connection_id, connection_type);
  }
  else if (typetag == "Event")
  {
    capabilities2_executor::node_t node;

    node.source.runner = nametag;
    node.source.provider = providertag;
    node.source.parameters = element;

    predecessor_id = connection_id - 1;

    while (connections.count(connection_id) > 0)
      connection_id += 1;

    connections[connection_id] = node;

    if ((connection_type == capabilities2_executor::connection_type_t::ON_SUCCESS) and (connection_id != 0))
      connections[predecessor_id].target_on_success = connections[connection_id].source;

    else if ((connection_type == capabilities2_executor::connection_type_t::ON_START) and (connection_id != 0))
      connections[predecessor_id].target_on_start = connections[connection_id].source;

    else if ((connection_type == capabilities2_executor::connection_type_t::ON_FAILURE) and (connection_id != 0))
      connections[predecessor_id].target_on_failure = connections[connection_id].source;

    if (hasSiblings)
      capabilities2_xml_parser::extract_connections(element->NextSiblingElement(), connections, connection_id + 1, connection_type);
  }
}

}  // namespace capabilities2_xml_parser
