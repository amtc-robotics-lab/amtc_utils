#pragma once
#include <exception>
#include <rcl_interfaces/msg/detail/set_parameters_result__struct.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rfl.hpp>
#include <rfl/enums.hpp>
#include <rfl/to_view.hpp>
#include <rfl/visit.hpp>
#include <type_traits>

namespace amtc{

    template< typename T>
    T declare_params(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface, std::string base_name ="")
    {
        T retval;
        const auto view = rfl::to_view(retval);
        view.apply([&parameter_interface, &base_name]<typename Field>( const Field& field){

            using field_type = std::remove_pointer_t<typename Field::Type>;

            std::string  name;
            if (base_name!="" ){
                if (base_name.back() == '.'){
                    name = base_name ;

                }else{
                    name = (base_name + '.');

                }
            }
            name.append(Field::name());



            if constexpr( std::is_same_v<field_type,int> ||
                std::is_same_v<field_type,double> ||
                std::is_same_v<field_type,bool> ||
                std::is_same_v<field_type,std::string> ||
                std::is_same_v<field_type,std::vector<int>> ||
                std::is_same_v<field_type,std::vector<double>> ||
                std::is_same_v<field_type,std::vector<std::string>> ||
                std::is_same_v<field_type,std::vector<bool>> ||
                std::is_same_v<field_type,std::vector<uint8_t>> ||
                std::is_same_v<field_type,std::vector<std::string>> ||
                std::is_same_v<field_type,std::vector<std::string>>
            ){
                // field.value() = parameter_interface->declare_parameter<field_type>(name);
                rclcpp::ParameterValue value{field_type{}};
                try{
                *field.value() = parameter_interface->declare_parameter(name , value.get_type()).get<field_type>();
                }catch (const rclcpp::ParameterTypeException &) {
                   throw rclcpp::exceptions::UninitializedStaticallyTypedParameterException(name);
                }
            }else if constexpr(std::is_enum<field_type>()){

                auto value = parameter_interface->declare_parameter(name,rclcpp::ParameterType::PARAMETER_STRING);
                auto result = rfl::string_to_enum<field_type>(value.get<std::string>());
                if (result){
                    *field.value() = *result;
                }else{
                    throw rclcpp::exceptions::InvalidParameterValueException("Parameter "+ name+"does not match enum requirement");
                }
            }
            else {
                *field.value() = declare_params<field_type>(parameter_interface, name);

            }

        });
        return retval;

    }

    template< typename T>
    T get_params(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface, std::string base_name ="")
    {
        T retval;
        const auto view = rfl::to_view(retval);
        view.apply([&parameter_interface, &base_name]<typename Field>( const Field& field){

            using field_type = std::remove_pointer_t<typename Field::Type>;

            std::string  name;
            if (base_name!="" ){
                if (base_name.back() == '.'){
                    name = base_name ;

                }else{
                    name = (base_name + '.');

                }
            }
            name.append(Field::name());


            if constexpr( std::is_same_v<field_type,int> ||
                std::is_same_v<field_type,double> ||
                std::is_same_v<field_type,bool> ||
                std::is_same_v<field_type,std::string> ||
                std::is_same_v<field_type,std::vector<int>> ||
                std::is_same_v<field_type,std::vector<double>> ||
                std::is_same_v<field_type,std::vector<std::string>> ||
                std::is_same_v<field_type,std::vector<bool>> ||
                std::is_same_v<field_type,std::vector<uint8_t>> ||
                std::is_same_v<field_type,std::vector<std::string>>
            ){
                // field.value() = parameter_interface->declare_parameter<field_type>(name);
                rclcpp::ParameterValue value{field_type{}};
                try{
                *field.value() = parameter_interface->get_parameter(name).get_value<field_type>();
                }catch (const rclcpp::ParameterTypeException &) {
                   throw rclcpp::exceptions::UninitializedStaticallyTypedParameterException(name);
                }
            }
            else if constexpr(std::is_enum_v<field_type>){
                auto value = rfl::string_to_enum<field_type>(parameter_interface->get_parameter(name).as_string());

                if (value){
                    *field.value() = *value;
                }
                else{
                    throw rclcpp::exceptions::InvalidParameterValueException("enum "+name+" did not match any enum option");
                }
            }
            else {
                *field.value() = get_params<field_type>(parameter_interface, name);

            }

        });
        return retval;

    }

    template <typename Field>
    void process_parameter_change(const Field& field, const rclcpp::Parameter &change, std::string base_name=""){
        using field_type = std::remove_pointer_t<typename Field::Type>;
        std::string  name;
        if (base_name!="" ){
            if (base_name.back() == '.'){
                name = base_name ;

            }else{
                name = (base_name + '.');

            }
        }
        name.append(Field::name());





                    if constexpr( std::is_same_v<field_type,int> ||
                        std::is_same_v<field_type,double> ||
                        std::is_same_v<field_type,bool> ||
                        std::is_same_v<field_type,std::string> ||
                        std::is_same_v<field_type,std::vector<int>> ||
                        std::is_same_v<field_type,std::vector<double>> ||
                        std::is_same_v<field_type,std::vector<std::string>> ||
                        std::is_same_v<field_type,std::vector<bool>> ||
                        std::is_same_v<field_type,std::vector<uint8_t>> ||
                        std::is_same_v<field_type,std::vector<std::string>>
                    ){
                        rclcpp::ParameterValue value{field_type{}};
                        if (change.get_name() == name){
                            if (change.get_type() == value.get_type()){
                                *field.value() = change.get_parameter_value().get<field_type>();
                            }else{
                                throw  rclcpp::ParameterTypeException(value.get_type(),change.get_type());
                            }
                        }
                    }
                    else if constexpr(std::is_enum_v<field_type>){

                        if (change.get_name() == name){
                                auto value = rfl::string_to_enum<field_type>(change.as_string());
                                if (value){
                                    *field.value() = *value;
                                }
                                else{
                                    throw rclcpp::exceptions::InvalidParameterValueException("enum "+name+" did not match any enum option");
                                }
                        }

                    }
                    else {
                        const auto view = rfl::to_view(*field.value());
                        view.apply([&change]<typename F> (const F& field){
                            process_parameter_change(field, change);
                        });


                    }
    }

    template< typename T>
 rcl_interfaces::msg::SetParametersResult     validate_param_changes(T prev_value, const std::vector<rclcpp::Parameter> &parameter_changes)
    {
        rcl_interfaces::msg::SetParametersResult retval;
        const auto view = rfl::to_view(prev_value);
        try{
        for (auto &change: parameter_changes){



            view.apply([&change]<typename Field>( const Field& field){
                process_parameter_change(field, change);
            });
        }

        }
        catch (std::exception &e){
            retval.successful = false;
            retval.reason = e.what();
            return retval;
        }

        retval.successful = true;
        return retval;

    }
}
