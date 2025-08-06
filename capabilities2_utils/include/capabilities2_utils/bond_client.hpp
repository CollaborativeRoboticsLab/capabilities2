#pragma once
#include <string>
#include <bondcpp/bond.hpp>
#include <rclcpp/rclcpp.hpp>
#include <event_logger/event_client.hpp>

class BondClient
{
public:
  /**
   * @brief Construct a new Bond Client object
   * 
   * @param node pointer to the node
   * @param event_client pointer to the event client
   * @param bond_id Bond id string
   * @param bonds_topic Bond topic to be published
   */
  BondClient(rclcpp::Node::SharedPtr node, std::shared_ptr<EventClient> event_client, const std::string &bond_id, const std::string &bonds_topic = "/capabilities/bond")
  {
    topic_ = bonds_topic;
    bond_id_ = bond_id;
    node_ = node;
    event_ = event_client;
  }

  /**
   * @brief start the bond
   * 
   */
  void start()
  {
    event_->info("[BondClient] creating bond to capabilities server");

    bond_ = std::make_unique<bond::Bond>(topic_, bond_id_, node_, std::bind(&BondClient::on_broken, this), std::bind(&BondClient::on_formed, this));

    bond_->setHeartbeatPeriod(0.10);
    bond_->setHeartbeatTimeout(10.0);
    bond_->start();
  }

  /**
   * @brief stop the bond
   * 
   */
  void stop()
  {
    event_->info("[BondClient] destroying bond to capabilities server");

    if (bond_)
    {
      bond_.reset();
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
    event_->info("[BondClient] bond with capabilities server formed with id: " + bond_id_);
  }

  /**
   * @brief callback function for bond broken event
   * 
   */
  void on_broken()
  {
    // log bond established event
    event_->info("[BondClient] bond with capabilities server broken with id: " + bond_id_);
  }

  /** Ros node pointer */
  rclcpp::Node::SharedPtr node_;

  /** Bond id string */
  std::string bond_id_;

  /** Bond topic to be published */
  std::string topic_;

  /** Heart beat bond with capabilities server */
  std::shared_ptr<bond::Bond> bond_;

  /** Event client for publishing events */
  std::shared_ptr<EventClient> event_;
};