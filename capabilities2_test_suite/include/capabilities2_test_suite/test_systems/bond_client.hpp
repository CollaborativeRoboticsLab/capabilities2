#pragma once
#include <string>
#include <bondcpp/bond.hpp>
#include <rclcpp/rclcpp.hpp>

class BondClient
{
public:
  /**
   * @brief Construct a new Bond Client object
   * 
   * @param node pointer to the node
   * @param bond_id Bond id string
   */
  BondClient(rclcpp::Node::SharedPtr node, const std::string &bond_id)
  {
    bond_id_ = bond_id;
    node_ = node;

    //get the bond topic from parameter server
    node_->declare_parameter<std::string>("capability_client.bond_topic", "/capabilities/bonds");
    node_->get_parameter("capability_client.bond_topic", topic_);
  }

  /**
   * @brief start the bond
   * 
   */
  void start()
  {
    RCLCPP_INFO(node_->get_logger(), "Creating bond to capabilities server with id: %s", bond_id_.c_str());

    bond_ = std::make_unique<bond::Bond>(topic_, bond_id_, node_, std::bind(&BondClient::on_broken, this), std::bind(&BondClient::on_formed, this));

    bond_->setHeartbeatPeriod(0.10);
    bond_->setHeartbeatTimeout(10.0);
    bond_->start();

    RCLCPP_INFO(node_->get_logger(), "Bond started with capabilities server with id: %s", bond_id_.c_str());
  }

  /**
   * @brief stop the bond
   * 
   */
  void stop()
  {
    RCLCPP_INFO(node_->get_logger(), "Destroying bond to capabilities server with id: %s", bond_id_.c_str());

    if (bond_)
    {
      bond_.reset();
      RCLCPP_INFO(node_->get_logger(), "Bond stopped with capabilities server with id: %s", bond_id_.c_str());
    }
  }

  ~BondClient()
  {
    stop();
  }

private:
  /**
   * @brief callback function for bond formed event
   * 
   */
  void on_formed()
  {
    // log bond established event
    RCLCPP_INFO(node_->get_logger(), "Bond with capabilities server formed with id: %s", bond_id_.c_str());
  }

  /**
   * @brief callback function for bond broken event
   * 
   */
  void on_broken()
  {
    // log bond established event
    RCLCPP_INFO(node_->get_logger(), "Bond with capabilities server broken with id: %s", bond_id_.c_str());
  }

  /** Ros node pointer */
  rclcpp::Node::SharedPtr node_;

  /** Bond id string */
  std::string bond_id_;

  /** Bond topic to be published */
  std::string topic_;

  /** Heart beat bond with capabilities server */
  std::shared_ptr<bond::Bond> bond_;
};