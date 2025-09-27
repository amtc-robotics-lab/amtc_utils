
#include "amtc_utils/params/params_helper.h"
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rfl/enums.hpp>
#include <rfl/json/write.hpp>


template <typename T>
void print_fields(const std::string base_name=""){
    T retval;
    const auto view = rfl::to_view(retval);
    view.apply([base_name]<typename Field>( const Field& field){
        using field_type = std::remove_pointer_t<typename Field::Type>;

        field_type a ;
        std::string  name;
        if (base_name!="" ){
            if (base_name.back() == '.'){
                name = base_name ;

            }else{
                name = (base_name + '.');

            }
        }
        name.append(Field::name());


        std::cout << "name : " << name << " val" <<"\n";
        if constexpr( std::is_same_v<field_type,int>){
            std::cout << name <<  " is an int\n";
        }
        else if constexpr (std::is_same_v<field_type, double>) {

            std::cout << name <<  " is a double\n";
        }
        else if constexpr (std::is_same_v<field_type, std::string>) {

            std::cout << name <<  " is a string\n";
        }
        else if constexpr (std::is_same_v<field_type, std::vector<double>>) {

            std::cout << name <<  " is a double array\n";
        }
        else if constexpr (std::is_same_v<field_type, std::vector<int>>) {

            std::cout << name <<  " is a int array\n";
        }
        else {
            std::cout << name <<  " is a unknown , interating into it\n";
            print_fields<field_type>(name);

        }


    });

}

int N=10;

class ParamsExample : public rclcpp::Node{

public:
    ParamsExample():  Node("params_example"){

        using namespace std::chrono_literals;

        config_ = amtc::declare_params<Config>(get_node_parameters_interface());
        timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(1.001), std::bind(&ParamsExample::timer_cb, this)  );

        parameter_callback_handle_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> &changes){

            auto retval = amtc::validate_change<Config>(config_, changes);
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
        // std::cout << "\n";
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    bool params_have_changed_ = false;

    enum struct EnumTest{red, green , orange};
    struct Config{
        std::string str_param;
        double d;
        int i;
        std::vector<double> da;
        EnumTest color;
        struct SubConfig{
            int test_int;
            double test_double;
        } sub_config;
    }config_;
};



int main(int argc, char** argv){

        enum struct EnumTest{red, green , orange};
        struct Config{
            std::string str_param;
            double d;
            int i;
            std::vector<double> da;
            EnumTest color;
            struct SubConfig{
                int test_int;
                double test_double;
            } sub_config;
        }config_;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamsExample>());

    config_.i = 30;
    config_.color =  EnumTest::green;
    enum Color{red, green} color= Color::green;
    std::cout << rfl::enum_to_string(color)<< "\n";
    std::string jsonstr = rfl::json::write(config_);
    std::cout << jsonstr<< "\n";
    rclcpp::shutdown();

    return 0;
}
