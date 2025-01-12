#include <bondcpp/bond.hpp>
#include <rclcpp/rclcpp.hpp>

class BondClient
{
public:
  BondClient(rclcpp::Node::SharedPtr node, const std::string& bond_id, const std::string& bonds_topic = "/capabilities/bond")
  {
    bonds_topic_ = bonds_topic;
    bond_id_ = bond_id;
    node_ = node;
  }

  void start()
  {
    RCLCPP_INFO(node_->get_logger(), "[BondClient] creating bond to capabilities server");

    bond_ =
        std::make_unique<bond::Bond>(bonds_topic_, bond_id_, node_, std::bind(&BondClient::on_broken, this), std::bind(&BondClient::on_formed, this));

    bond_->setHeartbeatPeriod(0.10);
    bond_->setHeartbeatTimeout(10.0);
    bond_->start();
  }

  void stop()
  {
    RCLCPP_INFO(node_->get_logger(), "[BondClient] destroying bond to capabilities server");

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
  void on_formed()
  {
    // log bond established event
    RCLCPP_INFO(node_->get_logger(), "[BondClient] bond with capabilities server formed with id: %s", bond_id_.c_str());
  }

  void on_broken()
  {
    // log bond established event
    RCLCPP_INFO(node_->get_logger(), "[BondClient] bond with capabilities server broken with id: %s", bond_id_.c_str());
  }

  /** Ros node pointer */
  rclcpp::Node::SharedPtr node_;

  /** Bond id string */
  std::string bond_id_;

  /** Bond topic to be published */
  std::string bonds_topic_;

  /** Heart beat bond with capabilities server */
  std::shared_ptr<bond::Bond> bond_;
};