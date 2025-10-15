
#include "amtc_utils/params_helper.h"
#include <exception>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rfl/Rename.hpp>
#include <rfl/Validator.hpp>
#include <rfl/enums.hpp>
#include <rfl/json.hpp>
#include <rfl/json/Writer.hpp>
#include <stdexcept>
#include <type_traits>

class ParamsExample : public rclcpp::Node {

public:
  ParamsExample() : Node("params_example") {

    using namespace std::chrono_literals;

    config_ = amtc::declare_params<Config>(get_node_parameters_interface());
    timer_ = rclcpp::create_timer(this, this->get_clock(),
                                  rclcpp::Duration::from_seconds(1.001),
                                  std::bind(&ParamsExample::timer_cb, this));

    parameter_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &changes) {
          auto retval = amtc::validate_param_changes<Config>(config_, changes);
          if (retval.successful) {
            params_have_changed_ = true;
            RCLCPP_INFO(get_logger(), "accepted parameter changes");
          } else {
            RCLCPP_WARN(get_logger(),
                        "Ignoring parameter change with errors: %s",
                        retval.reason.c_str());
          }

          return retval;
        });
  }

  void timer_cb() {
    if (params_have_changed_) {
      config_ = amtc::get_params<Config>(get_node_parameters_interface());
    }
    std::string jsonpar = rfl::json::write(config_);
    std::cout << "params::  " << jsonpar << "\n";
    std::cout << "\n";
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
  bool params_have_changed_ = false;

  enum struct EnumTest { red, green, orange };
  struct Config {
    std::string str_param;
    rfl::Rename<"d", double> d_new;
    rfl::Validator<int, rfl::Minimum<10>> i =
        10; // needs to have a valid default ( still won't have a default value
            // from ROS point of view, ros wont know about the range yet ,
            // hopefully in the future)

    std::vector<double> da;
    rfl::Validator<double,amtc::RosRange< 0.0, 10.0>> duration;
    rfl::Validator<double,amtc::RosRange< 0.0, 10.0, 1.0>> range_test_0;
    rfl::Validator<int,amtc::RosRange< 0, 20, 7>> range_test_int_0;
    EnumTest color;

    struct SubConfig {
      rfl::Validator<int, rfl::Minimum<10>> test_int = 2000;
      double test_double;
      enum Mode { autonomous, manual, teleop, assisted } mode;
    } sub_config;
  } config_;
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamsExample>());

  rclcpp::shutdown();

  return 0;
}
