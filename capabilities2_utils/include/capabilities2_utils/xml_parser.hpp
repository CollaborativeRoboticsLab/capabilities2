#pragma once
#include <tinyxml2.h>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <capabilities2_utils/connection.hpp>
#include <capabilities2_utils/status_client.hpp>

namespace xml_parser
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
   * @param element XMLElement element to be converted
   * @param paramters parameter to hold std::string
   *
   * @return `true` if element is not nullptr and conversion successful, `false` if element is nullptr
   */
  bool convert_to_string(tinyxml2::XMLElement *element, std::string &parameters)
  {
    if (element)
    {
      tinyxml2::XMLPrinter printer;
      element->Accept(&printer);
      parameters = printer.CStr();
      return true;
    }
    else
    {
      parameters = "";
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
  void convert_to_string(tinyxml2::XMLDocument &document_xml, std::string &document_string)
  {
    tinyxml2::XMLPrinter printer;
    document_xml.Print(&printer);
    document_string = printer.CStr();
  }

  /**
   * @brief check the plan for invalid/unsupported control and event tags
   * uses recursive approach to go through the plan
   *
   * @param status StatusClient used for logging and status publishing
   * @param element XML Element to be evaluated
   * @param events list containing valid event tags
   * @param providers list containing providers
   * @param control list containing valid control tags
   * @param rejected list containing invalid tags
   *
   * @return `true` if element valid and supported and `false` otherwise
   */
  bool check_tags(const std::shared_ptr<StatusClient> status, tinyxml2::XMLElement *element, std::vector<std::string> &events, std::vector<std::string> &providers,
                  std::vector<std::string> &control, std::vector<std::string> &rejected)
  {
    const char *name;
    const char *provider;

    std::string parameter_string;
    convert_to_string(element, parameter_string);

    element->QueryStringAttribute("name", &name);
    element->QueryStringAttribute("provider", &provider);

    std::string nametag;
    std::string providertag;

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
    bool hasSiblings = !xml_parser::isLastElement(element);
    bool foundInControl = xml_parser::search(control, nametag);
    bool foundInEvents = xml_parser::search(events, nametag);
    bool foundInProviders = xml_parser::search(providers, providertag);
    bool returnValue = true;

    if (typetag == "Control")
    {
      if (!foundInControl)
      {
        std::string msg = "Control tag '" + nametag + "' not available in the valid list";
        status->error(msg);
        rejected.push_back(parameter_string);
        return false;
      }

      if (hasChildren)
        returnValue &= xml_parser::check_tags(status, element->FirstChildElement(), events, providers, control, rejected);

      if (hasSiblings)
        returnValue &= xml_parser::check_tags(status, element->NextSiblingElement(), events, providers, control, rejected);
    }
    else if (typetag == "Event")
    {
      if (!foundInEvents || !foundInProviders)
      {
        std::string msg = "Event tag name '" + nametag + "' or provider '" + providertag + "' not available in the valid list";
        status->error(msg);
        rejected.push_back(parameter_string);
        return false;
      }

      if (hasSiblings)
        returnValue &= xml_parser::check_tags(status, element->NextSiblingElement(), events, providers, control, rejected);
    }
    else
    {
      std::string msg = "XML element is not valid :" + parameter_string;
      status->error(msg);
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
  void extract_connections(tinyxml2::XMLElement *element, std::map<int, capabilities2::node_t> &connections, int connection_id = 0,
                           capabilities2::connection_type_t connection_type = capabilities2::connection_type_t::ON_SUCCESS)
  {
    int predecessor_id;

    const char *name;
    const char *provider;

    element->QueryStringAttribute("name", &name);
    element->QueryStringAttribute("provider", &provider);

    std::string typetag(element->Name());

    std::string nametag;
    std::string providertag;

    if (name)
      nametag = name;
    else
      nametag = "";

    if (provider)
      providertag = provider;
    else
      providertag = "";

    bool hasChildren = !element->NoChildren();
    bool hasSiblings = !xml_parser::isLastElement(element);

    if ((typetag == "Control") and (nametag == "sequential"))
    {
      if (hasChildren)
        xml_parser::extract_connections(element->FirstChildElement(), connections, connection_id, capabilities2::connection_type_t::ON_SUCCESS);

      if (hasSiblings)
        xml_parser::extract_connections(element->NextSiblingElement(), connections, connection_id, connection_type);
    }
    else if ((typetag == "Control") and (nametag == "parallel"))
    {
      if (hasChildren)
        xml_parser::extract_connections(element->FirstChildElement(), connections, connection_id, capabilities2::connection_type_t::ON_START);

      if (hasSiblings)
        xml_parser::extract_connections(element->NextSiblingElement(), connections, connection_id, connection_type);
    }
    else if ((typetag == "Control") and (nametag == "recovery"))
    {
      if (hasChildren)
        xml_parser::extract_connections(element->FirstChildElement(), connections, connection_id, capabilities2::connection_type_t::ON_FAILURE);

      if (hasSiblings)
        xml_parser::extract_connections(element->NextSiblingElement(), connections, connection_id, connection_type);
    }
    else if (typetag == "Event")
    {
      capabilities2::node_t node;

      node.source.runner = nametag;
      node.source.provider = providertag;
      node.source.parameters = element;

      predecessor_id = connection_id - 1;

      while (connections.count(connection_id) > 0)
        connection_id += 1;

      connections[connection_id] = node;

      if ((connection_type == capabilities2::connection_type_t::ON_SUCCESS) and (connection_id != 0))
        connections[predecessor_id].target_on_success = connections[connection_id].source;

      else if ((connection_type == capabilities2::connection_type_t::ON_START) and (connection_id != 0))
        connections[predecessor_id].target_on_start = connections[connection_id].source;

      else if ((connection_type == capabilities2::connection_type_t::ON_FAILURE) and (connection_id != 0))
        connections[predecessor_id].target_on_failure = connections[connection_id].source;

      if (hasSiblings)
        xml_parser::extract_connections(element->NextSiblingElement(), connections, connection_id + 1, connection_type);
    }
  }

} // namespace xml_parser
