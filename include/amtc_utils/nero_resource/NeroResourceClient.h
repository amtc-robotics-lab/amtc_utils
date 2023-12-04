//
// Created by finostro on 27-11-23.
//

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <resource_manager_msgs/msg/resource_type.hpp>
#include <resource_manager_msgs/srv/free.hpp>
#include <resource_manager_msgs/srv/alloc.hpp>
//

/**
 * Class to request resources from the resource manager, and store the access token
 * @brief
 *
 */
class NeroResourceClient {
public:
  typedef std::shared_ptr< NeroResourceClient > SharedPtr;

  NeroResourceClient(rclcpp::Node *node, const char* resource_type);



  bool alloc(const rclcpp::Duration &timeout = rclcpp::Duration(1,0) );

  bool free(const rclcpp::Duration &timeout = rclcpp::Duration(1,0) );

  bool is_allocated();

  inline const std::string &get_access_token(){return access_token_;}

protected:
  rclcpp::Node * node_;
  std::string resource_type_;
  std::string access_token_;

  rclcpp::Client<resource_manager_msgs::srv::Alloc>::SharedPtr alloc_client_;
  rclcpp::Client<resource_manager_msgs::srv::Free>::SharedPtr free_client_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};