#include <amtc_utils/nero_resource/HeartbeatResource.h>

HeartbeatResource::HeartbeatResource(rclcpp::Node *node)
{
  node_ = node;

  configure();
  activate();
}

HeartbeatResource::~HeartbeatResource() {
}

void HeartbeatResource::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Heartbeat created");
  publisher_ = node_->create_publisher<HeartbeatMsg>(node_->get_name() + std::string("/heartbeat"), 10);
  hearbet_timer_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration::from_seconds(hearbeat_period_), std::bind(&HeartbeatResource::timer_cb, this));
}

void HeartbeatResource::configure()
{
  hearbeat_period_ = node_->declare_parameter<double>("RESOURCE.hearbeat_period");
}

void HeartbeatResource::timer_cb()
{
  HeartbeatMsg msg;
  msg.stamp = node_->get_clock()->now();
  msg.seq = heartbeat_seq_++;
  msg.node_name = node_->get_name();

  // RCLCPP_INFO(node_->get_logger(), "Send Heartbeat");

  publisher_->publish(msg);
}