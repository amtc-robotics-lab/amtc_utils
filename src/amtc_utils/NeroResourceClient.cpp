//
// Created by finostro on 27-11-23.
//
#include <amtc_utils/nero_resource/NeroResourceClient.h>



NeroResourceClient::NeroResourceClient(rclcpp::Node *node, const char* resource_type)
{
  node_ = node;
  resource_type_ = resource_type;
  access_token_ = "";
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  alloc_client_ = node_->create_client<resource_manager_msgs::srv::Alloc>("resource_manager/alloc", rmw_qos_profile_services_default, callback_group_);
  free_client_ = node_->create_client<resource_manager_msgs::srv::Free>("resource_manager/free", rmw_qos_profile_services_default, callback_group_);

}

bool NeroResourceClient::alloc(const rclcpp::Duration &timeout) {

    auto request = std::make_shared<resource_manager_msgs::srv::Alloc::Request>();
    request->resource.type = resource_type_;
    auto future = alloc_client_->async_send_request(request);
    auto future_status = future.wait_for(timeout.to_chrono<std::chrono::duration<int64_t, std::milli> >());

    if (future_status == std::future_status::timeout)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service resource_manager/alloc");

        return false;
    }
    else{
      auto result = future.get();
      if(!result->is_success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to allocate resource");
            return false;
        }
        access_token_ = result->token.data;
        return true;

    }
}

bool NeroResourceClient::free(const rclcpp::Duration &timeout) {
    auto request = std::make_shared<resource_manager_msgs::srv::Free::Request>();
    request->token.data = access_token_;
    auto result = free_client_->async_send_request(request);
    auto future_status = result.wait_for(timeout.to_chrono<std::chrono::duration<int64_t, std::milli> >());

    if (future_status == std::future_status::ready)
    {
        if(!result.get()->is_success)
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to free resource");
            return false;
        }

        access_token_ = "";

        return true;
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service resource_manager/free");
        return false;
    }
}
bool NeroResourceClient::is_allocated() {
    return !access_token_.empty();
}
