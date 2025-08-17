#pragma once
#include <string>

namespace capabilities2
{
enum event_t
{
  IDLE,
  STARTED,
  STOPPED,
  FAILED,
  SUCCEEDED
};

/**
 * @brief event definition
 *
 * Contains the informations about the event to be executed. It contains the interface, provider and parameters
 */
struct event
{
  std::string interface;
  std::string provider;
  std::string parameters;
};

/**
 * @brief event options
 *
 * keeps track of events that are related to runner instances at various points of the
 * plan
 * @param event_id unique identifier for the event
 * @param on_started information about the capability to execute on start
 * @param on_success information about the capability to execute on success
 * @param on_failure information about the capability to execute on failure
 * @param on_stopped information about the capability to execute on stop
 */
struct event_opts
{
  int event_id = -1;
  event on_started;
  event on_success;
  event on_failure;
  event on_stopped;
};

}