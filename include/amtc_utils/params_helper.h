#pragma once
#include <exception>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
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
#include <sstream>
#include <type_traits>

namespace amtc{

    std::vector<rclcpp::Parameter> declare_parameters(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface, std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors, std::string base_name ="", bool required = true);

    template<typename T>
    concept is_param = std::is_same_v<T,int> ||
                    std::is_same_v<T,double> ||
                    std::is_same_v<T,bool> ||
                    std::is_same_v<T,std::string> ||
                    std::is_same_v<T,std::vector<int>> ||
                    std::is_same_v<T,std::vector<double>> ||
                    std::is_same_v<T,std::vector<std::string>> ||
                    std::is_same_v<T,std::vector<bool>> ||
                    std::is_same_v<T,std::vector<uint8_t>> ||
                    std::is_same_v<T,std::vector<std::string>> ||
                    std::is_same_v<T,std::vector<std::string>>;



    template<typename T>
    concept has_reflection_type_defined = requires{
        typename T::ReflectionType;
    };

    template<typename T>
    concept has_type_defined = requires{
        typename T::Type;
    };

    template<typename T>
    concept is_convertible_to_param = is_param<T> ||
    ( has_type_defined<T> && is_param<typename T::Type>);

    template<typename T>
    concept has_enum_type = has_type_defined<T> && std::is_enum_v<typename  T::Type>;

    template<typename T>
   auto get_param_typeinfo(){
    if constexpr (has_reflection_type_defined<T>) {
        return std::type_identity<typename T::ReflectionType>{};
    }    else     if constexpr (has_type_defined<T>) {
         return std::type_identity<typename T::Type>{};
     }else{
     return std::type_identity<T>{};
     }
   };

    template<typename EnumType>
    std::string print_enum_names(){
        std::stringstream ss;
        ss<<"{";
        rfl::get_enumerators<EnumType>().apply([&ss](const auto &field) {
            ss<< field.name() << ",";
        });
        ss <<"}";
        return ss.str();
    }

    template< typename T>
    T declare_params(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_interface, std::string base_name ="")
    {
        T retval;
        const auto view = rfl::to_view(retval);
        view.apply([&parameter_interface, &base_name]<typename Field>( const Field& field){


            using original_type = std::remove_pointer_t<typename Field::Type>;
            using field_type = decltype(get_param_typeinfo< std::remove_pointer_t<typename Field::Type> >())::type;

            std::string  name;
            if (base_name!="" ){
                if (base_name.back() == '.'){
                    name = base_name ;

                }else{
                    name = (base_name + '.');

                }
            }
            name.append(Field::name());



            if constexpr( is_param<field_type>
            ){
                // field.value() = parameter_interface->declare_parameter<field_type>(name);
                try{
                    rclcpp::ParameterValue value{field_type{}};
                    *field.value() = parameter_interface->declare_parameter(name , value.get_type()).get<field_type>();
                }catch (const rclcpp::ParameterTypeException &) {
                   throw rclcpp::exceptions::UninitializedStaticallyTypedParameterException(name);
                }
            }
            else if constexpr(std::is_enum_v<field_type>){

                auto value = parameter_interface->declare_parameter(name,rclcpp::ParameterType::PARAMETER_STRING);
                auto result = rfl::string_to_enum<field_type>(value.get<std::string>());
                if (result){
                    *field.value() = *result;
                }else{
                    throw rclcpp::exceptions::InvalidParameterValueException(" Parameter "+ name+ " is an enum but  "+name+" does not match any enum option, options are "+ print_enum_names<field_type>());
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

            using field_type = decltype(get_param_typeinfo< std::remove_pointer_t<typename Field::Type> >())::type;



            std::string  name;
            if (base_name!="" ){
                if (base_name.back() == '.'){
                    name = base_name ;

                }else{
                    name = (base_name + '.');

                }
            }
            name.append(Field::name());


            if constexpr( is_convertible_to_param<field_type>
            ){
                // field.value() = parameter_interface->declare_parameter<field_type>(name);
                try{
                    rclcpp::ParameterValue value{field_type{}};
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
                    throw rclcpp::exceptions::InvalidParameterValueException(" Parameter "+ name+ " is an enum but  "+name+" does not match any enum option, options are "+ print_enum_names<field_type>());
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
        using field_type = decltype(get_param_typeinfo< std::remove_pointer_t<typename Field::Type> >())::type;
        std::string  name;
        if (base_name!="" ){
            if (base_name.back() == '.'){
                name = base_name ;

            }else{
                name = (base_name + '.');

            }
        }
        name.append(Field::name());






                    if constexpr( is_convertible_to_param<field_type>
                    ){

                        if (change.get_name() == name){
                            rclcpp::ParameterValue value{field_type{}};
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
                                    throw rclcpp::exceptions::InvalidParameterValueException(" Parameter "+ name+ " is an enum but  "+name+" does not match any enum option, options are "+ print_enum_names<field_type>());
                                }
                        }

                    }
                    else {
                        const auto view = rfl::to_view(*field.value());
                        view.apply([&change, &name]<typename F> (const F& field){
                            process_parameter_change(field, change, name);
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
