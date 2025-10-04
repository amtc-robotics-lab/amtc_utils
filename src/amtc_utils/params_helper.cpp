#include <amtc_utils/params_helper.h>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>

namespace amtc {

std::vector<rclcpp::Parameter> declare_parameters(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
        parameter_interface,
    std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors,
    std::string base_name, bool required) {
  std::vector<rclcpp::Parameter> retval;
  retval.reserve(descriptors.size());
  for (auto desc : descriptors) {

    if (!base_name.empty()) {
      desc.name = base_name + "." + desc.name;
    }

    try {

      retval.emplace_back(
          desc.name, parameter_interface->declare_parameter(
                         desc.name, rclcpp::ParameterType(desc.type), desc));
    } catch (
        rclcpp::exceptions::UninitializedStaticallyTypedParameterException &e) {
      continue;
    }
  }
  return retval;
};
} // namespace amtc
