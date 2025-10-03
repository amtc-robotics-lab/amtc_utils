
#include "amtc_utils/params_helper.h"
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rfl/enums.hpp>
#include <rfl/json.hpp>



class ParamsExample : public rclcpp::Node{

public:
    ParamsExample():  Node("params_example"){

        using namespace std::chrono_literals;

        config_ = amtc::declare_params<Config>(get_node_parameters_interface());
        timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1.001), std::bind(&ParamsExample::timer_cb, this)  );

        parameter_callback_handle_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &changes){

            auto retval = amtc::validate_param_changes<Config>(config_, changes);
            if (retval.successful){
                params_have_changed_ = true;
            }
            else{
                RCLCPP_WARN(get_logger(), "Ignoring parameter change with errors: %s" ,retval.reason.c_str());
            }
            return retval;
        });
    }

    void timer_cb(){
        if ( params_have_changed_){
            config_ = amtc::get_params<Config>(get_node_parameters_interface());
        }
        std::string jsonpar = rfl::json::write(config_);
        std::cout <<  "params::  " << jsonpar <<"\n";
        std::cout << "\n";
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    bool params_have_changed_ = false;

    enum struct EnumTest{red, green, orange};
    struct Config{
        std::string str_param;
        double d;
        int i;
        std::vector<double> da;
        double duration;
        EnumTest color;

        struct SubConfig{
            int test_int;
            double test_double;
            enum Mode{autonomous, manual, teleop, assisted}
            mode;
        } sub_config;
    }config_;
};



int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamsExample>());

    rclcpp::shutdown();

    return 0;
}
