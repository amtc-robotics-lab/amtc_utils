#pragma once

#include <rclcpp/rclcpp.hpp>
#include <autonomous_machine_interface/msg/heartbeat.hpp>

// resource server to send hearbeat msgs

using HeartbeatMsg = autonomous_machine_interface::msg::Heartbeat;

class HeartbeatResource{
public:
  typedef std::shared_ptr< HeartbeatResource > SharedPtr;

  HeartbeatResource(rclcpp::Node *node);
  ~HeartbeatResource();

private:

	void activate();
	void configure();
  void timer_cb();

  rclcpp::Node * node_;

  rclcpp::Publisher<HeartbeatMsg>::SharedPtr publisher_;

	double hearbeat_period_;
  long heartbeat_seq_;
};