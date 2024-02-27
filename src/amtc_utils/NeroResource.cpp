//
// Created by finostro on 27-11-23.
//
#include <amtc_utils/nero_resource/NeroResource.h>

NeroResource::NeroResource(rclcpp::Node *node, const char* resource_type):
        is_registered_(false)
{
  node_ = node;
  resource_type_ = resource_type;
  access_token_ = "";
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  register_client_ = node_->create_client<resource_manager_msgs::srv::Register>("resource_manager/register", rmw_qos_profile_services_default, callback_group_);
  unregister_client_ = node_->create_client<resource_manager_msgs::srv::Unregister>("resource_manager/unregister",rmw_qos_profile_services_default, callback_group_);

  alloc_service_name_ = std::string(node_->get_name())+"/"+ resource_type_+"_resource/alloc";
  free_service_name_ = std::string(node_->get_name())+"/"+ resource_type_+"_resource/free";

  alloc_token_service_ = node_->create_service<resource_manager_msgs::srv::Notify>(alloc_service_name_, std::bind(&NeroResource::alloc_token_cb, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
  free_token_service_ = node_->create_service<resource_manager_msgs::srv::Notify>(free_service_name_, std::bind(&NeroResource::free_token_cb, this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default, callback_group_);


  if (!register_resource()){
    throw std::runtime_error("Failed to register resource");
  }

}
NeroResource::~NeroResource() {
    if (is_registered_) {
      unregister_resource();
    }
}
void NeroResource::registered_cb(const std::shared_future<resource_manager_msgs::srv::Register::Response::SharedPtr> response){

    if (!response.valid()){
        RCLCPP_ERROR(node_->get_logger(), "Failed to register resource");
        return;
    }
    if (!response.get()->is_success) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to register resource");
      return;
    }
    provider_token_ = response.get()->provider_token.data;

    RCLCPP_INFO(node_->get_logger(), "Provider for type registered %s , token is %s", resource_type_.c_str(), access_token_.c_str());
    is_registered_ = true;
}

bool NeroResource::register_resource(const rclcpp::Duration &timeout) {
    auto request = std::make_shared<resource_manager_msgs::srv::Register::Request>();
    request->type.type = resource_type_;
    request->alloc_token_callback_service_name = alloc_service_name_;
    request->free_token_callback_service_name = free_service_name_;
    request->provider_name = std::string(node_->get_name())+"/"+resource_type_;

    RCLCPP_INFO(node_->get_logger(),"Waiting for resource_manager/register service to be available");
    while (!register_client_->wait_for_service(timeout.to_chrono<std::chrono::duration<int64_t, std::milli> >()) & rclcpp::ok()){
      RCLCPP_ERROR(node_->get_logger(), "Waiting for resource_manager/register service to be available");
    }
    RCLCPP_INFO(node_->get_logger(),"resource_manager/register service is available");
    register_future_ = register_client_->async_send_request(request, std::bind(&NeroResource::registered_cb, this, std::placeholders::_1));
    return true;

}

bool NeroResource::unregister_resource(const rclcpp::Duration &timeout) {
    if(!unregister_client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to find unregister server");
      return false;
    }
    auto request = std::make_shared<resource_manager_msgs::srv::Unregister::Request>();
    request->token.data = provider_token_;
    auto result = unregister_client_->async_send_request(request);
    auto future_status = result.wait_for(timeout.to_chrono<std::chrono::duration<int64_t, std::milli> >());
    if (future_status == std::future_status::ready)
    {
        if(!result.get()->is_success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to unregister resource");
            return false;
        }

        provider_token_ = "";
        access_token_ = "";
        is_registered_ = false;
        return true;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service resource_manager/unregister");
        return false;
    }
}

bool NeroResource::is_registered() {
    return is_registered_;
}


bool NeroResource::alloc_token_cb(resource_manager_msgs::srv::Notify::Request::SharedPtr req, resource_manager_msgs::srv::Notify::Response::SharedPtr res)
{
  access_token_ = req->subscriber_token.data;
  res->is_success = true;
  RCLCPP_INFO(node_->get_logger(), "Selected Body Controller token is %s", access_token_.c_str());



  return true;
}

bool NeroResource::free_token_cb(resource_manager_msgs::srv::Notify::Request::SharedPtr req,
                                 resource_manager_msgs::srv::Notify::Response::SharedPtr res) {


    if (req->subscriber_token.data == access_token_)
    {
        RCLCPP_INFO(node_->get_logger(), "Freed Body Controller token");
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Wrong token %s, will free active token %s anyway", req->subscriber_token.data.c_str(), access_token_.c_str());
    }

  access_token_ = "";
  res->is_success = true;
  return true;
}