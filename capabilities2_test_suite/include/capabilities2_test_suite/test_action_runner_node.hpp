#pragma once

#include <rclcpp/rclcpp.hpp>
#include <capabilities2_runner/CapabilityOptions.hpp>
#include <capabilities2_test_suite/test_systems/test_action_server.hpp>
#include <capabilities2_test_suite/test_systems/capabilities2_client.hpp>
#include <capabilities2_test_suite/test_systems/bond_client.hpp>

namespace capabilities2
{
class ActionTestCapabilities2Client: public rclcpp::Node
{
public:
  ActionTestCapabilities2Client(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
  : Node("action_test_capabilities2_client", options)
  {
    capability_client_ = std::make_shared<TestCapabilityClient>();
    capability_client_->initialize(this->shared_from_this());
  }

  void initialize()
  {
    capability_client_->getInterfaces(capability_list_);
    capability_client_->getSemanticInterfaces(capability_list_);
    capability_client_->getProviders(capability_list_);
  }

  bool test()
  {
    // request bond
    std::string bond_id = capability_client_->requestBond();

		// establish bond with the bond client
		bond_client_ = std::make_shared<BondClient>(this->shared_from_this(), bond_id);

		// create the capability option
		CapabilityOptions capability;
		capability.interface = "capabilities2_test_suite/TestActionRunner";
		capability.provider = "capabilities2_test_suite/TestActionRunner";
		capability.options.set_value("order", OptionType::INTEGER, 10);

		// check if the capability is available
		for (const auto& cap : capability_list_)
		{
			bool found = false;
			if (cap.interface == capability.interface && cap.provider == capability.provider)
			{
				RCLCPP_INFO(this->get_logger(), "capability found: " + capability.interface + " provided by " + capability.provider);
				found = true;
			}
		}

		// if the capability is not found then return false
		if (!found)
		{
			RCLCPP_ERROR(this->get_logger(), "capability not found: " + capability.interface + " provided by " + capability.provider);
			return false;
		}

		// use capability with the bond
		capability_client_->use_capability(capability, bond_id);

    // trigger capability with the bond
    capability_client_->trigger_capability(capability);
    return true;
  }

private:
  std::shared_ptr<BondClient> bond_client_;
  std::shared_ptr<TestCapabilityClient> capability_client_;
  std::vector<CapabilityInfo> capability_list_;
}; 
} // namespace capabilities2