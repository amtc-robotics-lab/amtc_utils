//
// Created by finostro on 27-11-23.
//

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <resource_manager_msgs/msg/resource_type.hpp>
#include <resource_manager_msgs/srv/register.hpp>
#include <resource_manager_msgs/srv/unregister.hpp>
#include <resource_manager_msgs/srv/notify.hpp>


// resource server to connect to resource manager

class NeroResource{
public:
  typedef std::shared_ptr< NeroResource > SharedPtr;

  NeroResource(rclcpp::Node *node, const char* resource_type);

  ~NeroResource();

  bool is_registered();

  inline const std::string &get_access_token(){return access_token_;}

private:


  void registered_cb(const std::shared_future<resource_manager_msgs::srv::Register::Response::SharedPtr> response);

  bool register_resource( const rclcpp::Duration &timeout = rclcpp::Duration(10,0) );

  bool unregister_resource( const rclcpp::Duration &timeout = rclcpp::Duration(10,0) );

  bool alloc_token_cb(resource_manager_msgs::srv::Notify::Request::SharedPtr req, resource_manager_msgs::srv::Notify::Response::SharedPtr res);

  bool free_token_cb(resource_manager_msgs::srv::Notify::Request::SharedPtr req, resource_manager_msgs::srv::Notify::Response::SharedPtr res);

  rclcpp::Node * node_;
  std::string resource_type_;
  std::string access_token_;
  std::string provider_token_; // token representing the resource provider (this node)


  std::string free_service_name_;
  std::string alloc_service_name_;

  bool is_registered_;

  rclcpp::Client<resource_manager_msgs::srv::Register>::SharedPtr register_client_;
  rclcpp::Client<resource_manager_msgs::srv::Unregister>::SharedPtr unregister_client_;
  std::optional<rclcpp::Client<resource_manager_msgs::srv::Register>::SharedFutureAndRequestId> register_future_;


  rclcpp::Service<resource_manager_msgs::srv::Notify>::SharedPtr alloc_token_service_;
  rclcpp::Service<resource_manager_msgs::srv::Notify>::SharedPtr free_token_service_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;



};