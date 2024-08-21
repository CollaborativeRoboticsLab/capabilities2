#pragma once

#include <map>
#include <string>
#include <vector>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <bondcpp/bond.hpp>

namespace capabilities2_server
{

/**
 * @brief bond cache class
 * keep track of bonds established by clients and assosciated resources
 *
 */
class BondCache
{
public:
  BondCache(const std::string& bonds_topic = "~/bonds") : bonds_topic_(bonds_topic)
  {
  }

  void add_bond(const std::string& capability, const std::string& bond_id)
  {
    // if capability is not in cache, add it
    if (bond_cache_.find(capability) == bond_cache_.end())
    {
      bond_cache_[capability] = { bond_id };
    }
    else
    {
      // if capability is in cache, append only new bond id
      if (std::find(bond_cache_[capability].begin(), bond_cache_[capability].end(), bond_id) ==
          bond_cache_[capability].end())
      {
        bond_cache_[capability].push_back(bond_id);
      }
    }
  }

  void remove_bond(const std::string& bond_id)
  {
    // remove bond id from all capability entries
    for (auto& [capability, bonds] : bond_cache_)
    {
      auto it = std::find(bonds.begin(), bonds.end(), bond_id);
      if (it != bonds.end())
      {
        bonds.erase(it);
      }
    }

    // remove empty capability entries
    for (auto it = bond_cache_.begin(); it != bond_cache_.end();)
    {
      if (it->second.empty())
      {
        it = bond_cache_.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }

  void remove_bond(const std::string& capability, const std::string& bond_id)
  {
    // remove bond id from capability entry
    auto it = std::find(bond_cache_[capability].begin(), bond_cache_[capability].end(), bond_id);
    if (it != bond_cache_[capability].end())
    {
      bond_cache_[capability].erase(it);
    }

    // remove capability entry if empty
    if (bond_cache_[capability].empty())
    {
      bond_cache_.erase(capability);
    }
  }

  // get all capabilities requested by a bond
  const std::vector<std::string> get_capabilities(const std::string& bond_id)
  {
    std::vector<std::string> capabilities;
    for (auto& [capability, bonds] : bond_cache_)
    {
      if (std::find(bonds.begin(), bonds.end(), bond_id) != bonds.end())
      {
        capabilities.push_back(capability);
      }
    }

    return capabilities;
  }

  // get all current bonds for a capability
  const std::vector<std::string> get_bonds(const std::string& capability)
  {
    return bond_cache_[capability];
  }

  // start a live bond
  void start(const std::string& bond_id, rclcpp::Node::SharedPtr node, std::function<void(void)> on_broken,
             std::function<void(void)> on_formed)
  {
    // create a new bond with event callbacks
    live_bonds_[bond_id] = std::make_unique<bond::Bond>(bonds_topic_, bond_id, node, on_broken, on_formed);

    // start the bond
    live_bonds_[bond_id]->start();
  }

private:
  // bonds topic
  std::string bonds_topic_;

  // bond cache
  // capability -> bond ids
  std::map<std::string, std::vector<std::string>> bond_cache_;

  // live bonds
  // bond id -> bond object
  std::map<std::string, std::unique_ptr<bond::Bond>> live_bonds_;
};

}  // namespace capabilities2_server
