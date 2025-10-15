
#include "amtc_utils/change_parameters_interface.h"
#include "amtc_utils/params_helper.h"
#include <memory>
#include <rclcpp/executor.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rfl/Rename.hpp>
#include <rfl/Validator.hpp>
#include <rfl/comparisons.hpp>
#include <rfl/enums.hpp>
#include <rfl/json.hpp>
#include <rfl/json/Writer.hpp>

using namespace std::chrono_literals;

class Changer : public rclcpp::Node {
public:
  Changer() : Node("params_changer") {

    parameter_basenames_         = {"light", "heavy", "simple"};
    change_parameters_interface_ = std::make_shared<amtc::ChangeParametersInterface>(
        get_node_base_interface(), get_node_graph_interface(), get_node_parameters_interface(),
        get_node_services_interface(), get_node_logging_interface(), "params_example", parameter_basenames_);

    timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(5.0),
                                  std::bind(&Changer::timer_cb, this));
  }

  void timer_cb() {
    if (!initialized_) {
      change_parameters_interface_->configure();
      change_parameters_interface_->activate();
      initialized_ = true;
    }
    current_params = (current_params + 1) % parameter_basenames_.size();
    RCLCPP_INFO_STREAM(get_logger(), "Changing params to " << parameter_basenames_[current_params]);
    change_parameters_interface_->switch_to_parameters(parameter_basenames_[current_params]);
  }

  bool initialized_ = false;
  std::shared_ptr<amtc::ChangeParametersInterface> change_parameters_interface_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::string> parameter_basenames_;
  size_t current_params = 0;
};

class ParamsExample : public rclcpp::Node {

public:
  ParamsExample() : Node("params_example") {

    RCLCPP_INFO_STREAM(get_logger(), "p init start");

    config_                    = amtc::declare_params<Config>(get_node_parameters_interface(), "params_example");
    timer_                     = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1.001),
                                                      std::bind(&ParamsExample::timer_cb, this));

    parameter_callback_handle_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &changes) {
      auto retval = amtc::validate_param_changes<Config>(config_, changes, "params_example");
      if (retval.successful) {
        params_have_changed_ = true;
        RCLCPP_INFO(get_logger(), "accepted parameter changes");
      } else {
        RCLCPP_WARN(get_logger(), "Ignoring parameter change with errors: %s", retval.reason.c_str());
      }

      return retval;
    });
    RCLCPP_INFO_STREAM(get_logger(), "p init start");
  }

  void timer_cb() {
    if (params_have_changed_) {
      config_ = amtc::get_params<Config>(get_node_parameters_interface(), "params_example");
    }

    std::string jsonpar = rfl::json::write(config_);
    RCLCPP_INFO_STREAM(get_logger(), "params::  " << jsonpar);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  bool params_have_changed_ = false;

  enum struct EnumTest { red, green, orange };
  struct Config {
    std::string str_param;
    rfl::Rename<"d", double> d_new;
    rfl::Validator<int, rfl::Minimum<10>> i = 10; // needs to have a valid default ( still won't have a default value
                                                  // from ROS point of view, ros wont know about the range yet ,
                                                  // hopefully in the future)
    std::vector<double> da;
    rfl::Validator<double, amtc::RosRange<0.0, 10.0>> duration;
    rfl::Validator<double, amtc::RosRange<0.0, 10.0, 1.0>> range_test_0;
    rfl::Validator<int, amtc::RosRange<0, 20, 7>> range_test_int_0;
    EnumTest color;

    struct SubConfig {
      int test_int;
      double test_double;
      enum Mode { autonomous, manual, teleop, assisted } mode;
    } sub_config;
  } config_;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 6);
  auto params_example = std::make_shared<ParamsExample>();
  auto changer        = std::make_shared<Changer>();
  executor.add_node(params_example);
  executor.add_node(changer);
  executor.spin();
  rclcpp::shutdown();

  return 0;
}
