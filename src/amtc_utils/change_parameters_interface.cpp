#include <algorithm>
#include <amtc_utils/change_parameters_interface.h>

#include <amtc_utils/params_helper.h>
#include <amtc_utils/Utils.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <iterator>
#include <memory>
#include <numeric>
#include <ranges>
#include <rcl_interfaces/srv/detail/describe_parameters__struct.hpp>
#include <rcl_interfaces/srv/detail/list_parameters__struct.hpp>
#include <rcl_interfaces/srv/detail/set_parameters_atomically__struct.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/create_client.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_interfaces/get_node_base_interface.hpp>
#include <rclcpp/node_interfaces/get_node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rmw/qos_profiles.h>

namespace amtc {

using namespace std::chrono_literals; // For use times as 1s, 2s, etc. in services

std::vector<rcl_interfaces::msg::ParameterDescriptor>
copy_if_prefix(const std::vector<rcl_interfaces::msg::ParameterDescriptor> &in, const std::string_view prefix) {
  std::vector<rcl_interfaces::msg::ParameterDescriptor> out;
  out.reserve(in.size() - 4);
  std::ranges::copy_if(in, std::back_inserter(out), [&prefix](const auto &p) {
    return p.name.starts_with(prefix);
  });
  return out;
}

ChangeParametersInterface::ChangeParametersInterface(rclcpp::Node &node, std::string node_name, std::vector<std::string> basenames):
ChangeParametersInterface(node.get_node_base_interface(),node.get_node_graph_interface(), node.get_node_parameters_interface(),
                          node.get_node_services_interface(), node.get_node_logging_interface(),node_name,basenames)
{
}


ChangeParametersInterface::ChangeParametersInterface(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface,
    rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, std::string node_name,
    std::vector<std::string> parameter_namespaces)
    : base_interface_(node_base), graph_interface_(node_graph), parameter_interface_(parameter_interface),
      services_interface_(services_interface), logging_interface_(logging_interface), node_name_(node_name),
      parameter_namespaces_(parameter_namespaces), logger_(logging_interface->get_logger()) {}


std::vector<rclcpp::Parameter>& ChangeParametersInterface::get_parameters(std::string &basename){
  return parameter_set_.at(basename);
}

bool ChangeParametersInterface::switch_to_parameters(const std::vector<rclcpp::Parameter> &parameters) {

  auto req = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();

  req->parameters.resize(parameters.size());
  for (size_t i = 0; i < parameters.size(); i++) {
    req->parameters[i] = parameters[i].to_parameter_msg();
  }
  auto future = set_parameters_client_->async_send_request(std::move(req));

  if (!(future.wait_for(0.1s) == std::future_status::ready)) {
    RCLCPP_ERROR_STREAM(logging_interface_->get_logger(), "Could not set params to node " << node_name_);
    return false;
  }

  auto result         = future.get()->result;
  bool change_mode_ok = result.successful;
  if (change_mode_ok) {
    RCLCPP_INFO_STREAM(logging_interface_->get_logger(), "Successfully set paramterse to node " << node_name_);
    return true;
  }
  RCLCPP_ERROR_STREAM(logging_interface_->get_logger(),
                      "Could not set " << node_name_ << " Params reason : " << result.reason);
  return false;
}

bool ChangeParametersInterface::switch_to_parameters(const std::string &basename) {

  return switch_to_parameters(parameter_set_[basename]);
}

void ChangeParametersInterface::activate() {
  // we are ready since configure
  RCLCPP_INFO(logging_interface_->get_logger(), "change param interface activating");
  RCLCPP_INFO(logging_interface_->get_logger(), "waiting for services");

  wait_for_service(describe_parameters_client_, logger_);
  wait_for_service(set_parameters_client_, logger_);
  wait_for_service(list_parameters_client_, logger_);


  RCLCPP_INFO(logging_interface_->get_logger(), "getting descriptions");
  get_parameter_descriptions();
  RCLCPP_INFO(logging_interface_->get_logger(), "getting parameters");
  get_parameter_set();
  RCLCPP_INFO(logging_interface_->get_logger(), "activated");
}

void ChangeParametersInterface::configure() {
  RCLCPP_INFO_STREAM(logging_interface_->get_logger(), "Configuring parameter interface for  node : " << node_name_);
  callback_group_             = base_interface_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  describe_parameters_client_ = rclcpp::create_client<rcl_interfaces::srv::DescribeParameters>(
      base_interface_, graph_interface_, services_interface_, node_name_ + "/describe_parameters",
      rmw_qos_profile_services_default, callback_group_);
  set_parameters_client_ = rclcpp::create_client<rcl_interfaces::srv::SetParametersAtomically>(
      base_interface_, graph_interface_, services_interface_, node_name_ + "/set_parameters_atomically",
      rmw_qos_profile_services_default, callback_group_);
  list_parameters_client_ = rclcpp::create_client<rcl_interfaces::srv::ListParameters>(
      base_interface_, graph_interface_, services_interface_, node_name_ + "/list_parameters",
      rmw_qos_profile_services_default, callback_group_);
}

void ChangeParametersInterface::get_parameter_set() {
  parameter_set_["default"]  = declare_parameters(parameter_interface_, parameter_descriptions_);
  std::string default_prefix = node_name_ + ".";

  for (auto &name : parameter_namespaces_) {

    std::string new_prefix        = node_name_ + "." + name + ".";

    auto msg_view                 = std::views::transform(parameter_set_["default"], [&default_prefix](auto p) {
      auto v = p.to_parameter_msg();
      if (v.name.starts_with(default_prefix)) {
        v.name = v.name.substr(default_prefix.size());
      }
      return v;
    });
    auto change_prefix_to_default = [&default_prefix, &new_prefix](rclcpp::Parameter p) {
      std::string new_name;
      if (p.get_name().starts_with(new_prefix)) {
        new_name = default_prefix + p.get_name().substr(new_prefix.size());
      } else {
        new_name = p.get_name();
      }
      rclcpp::Parameter retval(new_name, p.get_parameter_value());
      return retval;
    };
    std::vector<rcl_interfaces::msg::Parameter> scoped_values_msg(msg_view.begin(), msg_view.end());
    std::vector<rclcpp::Parameter> parameters;
    std::ranges::transform(declare_parameters(parameter_interface_, scoped_values_msg, node_name_ + "." + name),
                           std::back_inserter(parameters), change_prefix_to_default);
    parameter_set_[name] = parameters;
  }
}

void ChangeParametersInterface::get_parameter_descriptions() {
  auto req                       = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
  req->depth                     = rcl_interfaces::srv::ListParameters_Request::DEPTH_RECURSIVE;
  RCLCPP_INFO(logging_interface_->get_logger(), "listing params");

  auto future                    = list_parameters_client_->async_send_request(req);
  RCLCPP_INFO(logging_interface_->get_logger(), "waiting");
  while (!(future.wait_for(0.5s) == std::future_status::ready)) {
    RCLCPP_INFO_STREAM(logging_interface_->get_logger(), "List parameter not responding, recalling " << node_name_);
    future                    = list_parameters_client_->async_send_request(req);
  }
  std::vector<std::string> names = future.get()->result.names;
  auto req_describe_params       = std::make_shared<rcl_interfaces::srv::DescribeParameters::Request>();
  req_describe_params->names     = names;
  RCLCPP_INFO(logging_interface_->get_logger(), "desc");
  auto future_get_params         = describe_parameters_client_->async_send_request(req_describe_params);
  RCLCPP_INFO(logging_interface_->get_logger(), "waiting");
  while (!(future_get_params.wait_for(0.5s) == std::future_status::ready)) {
    RCLCPP_INFO_STREAM(logging_interface_->get_logger(), "describe parameter not responding, recalling " << node_name_);
    future_get_params         = describe_parameters_client_->async_send_request(req_describe_params);
  }

  auto descriptions              = future_get_params.get()->descriptors;
  RCLCPP_INFO(logging_interface_->get_logger(), "got desc");

  parameter_descriptions_        = copy_if_prefix(descriptions, node_name_);
  RCLCPP_INFO(logging_interface_->get_logger(), "done");
}

} // namespace amtc
