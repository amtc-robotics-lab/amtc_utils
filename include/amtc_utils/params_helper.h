#pragma once

#include <exception>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include <rcl_interfaces/msg/detail/parameter_type__struct.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rfl.hpp>
#include <rfl/enums.hpp>
#include <rfl/parsing/schema/ValidationType.hpp>
#include <rfl/to_view.hpp>
#include <rfl/visit.hpp>
#include <rfl/internal/is_validator.hpp>
#include <sstream>
#include <type_traits>

namespace amtc {


template <auto _minimum, auto _maximum, auto _step = 0>
  struct RosRange {
      static constexpr auto minimum =  _minimum;
      static constexpr auto maximum =  _maximum;
      static constexpr auto step =  _step;
  template <class T>
  static rfl::Result<T> validate(T _value) noexcept {
      static constexpr T minimum =  static_cast<T>(_minimum);
      static constexpr T maximum =  static_cast<T>(_maximum);
      static constexpr T step =  static_cast<T>(_step);
    if (_value < minimum || _value > maximum) {
      std::stringstream stream;
      stream << "Value outside of expected range of [" << minimum << " , "
             << maximum << "], but got " << _value << ".";
      return rfl::error(stream.str());
    }else if constexpr (step!=0) {
        if constexpr (std::is_floating_point_v<T>) {
            if (_value!=maximum && std::remainder((_value-minimum), step) !=0){
                std::stringstream stream;
                stream << "value "<< _value << " does not comply with step restriction min,max,step: "<<
                       minimum << " , " << maximum << " ,  " << step << ".";
                return rfl::error(stream.str());

            }
        }else {
            if (_value!=maximum && ((_value-minimum) %step) !=0){
                std::stringstream stream;
                stream << "value "<< _value << " does not comply with step restriction min,max,step: "<<
                    minimum << " , " << maximum << " ,  " << step << ".";
                return rfl::error(stream.str());

            }
        }

    }

    return _value;
  }

  template <class T>
  static rfl::parsing::schema::ValidationType to_schema() {
    using ValidationType = rfl::parsing::schema::ValidationType;
    const auto min_value =
        std::is_same_v<T,double>
            ? rfl::Variant<double, int>(static_cast<double>(_minimum))
            : rfl::Variant<double, int>(static_cast<int>(_minimum));
    rfl::parsing::schema::ValidationType min_validation_type{.variant_=rfl::parsing::schema::ValidationType::Minimum{min_value}};

    const auto max_value =
        std::is_same_v<T,int>
            ? rfl::Variant<double, int>(static_cast<double>(_maximum))
            : rfl::Variant<double, int>(static_cast<int>(_maximum));
    rfl::parsing::schema::ValidationType max_validation_type{.variant_=rfl::parsing::schema::ValidationType::Maximum{max_value}};
    if constexpr (step==0) {

        return rfl::parsing::schema::ValidationType{.variant_=rfl::parsing::schema::ValidationType::AllOf{.types_{min_validation_type, max_validation_type}}};
    }else{

        return rfl::parsing::schema::ValidationType{.variant_=rfl::parsing::schema::ValidationType::AllOf{.types_{min_validation_type, max_validation_type}}};
    }

  }
};

template <class T>
class is_ros_range;

template <class T>
class is_ros_range: public std::false_type{};

template < auto _minimum, auto _maximum, auto step>
class is_ros_range<RosRange< _minimum, _maximum, step>>: public std::true_type{};

template <class T>
constexpr bool is_ros_range_v =
    is_ros_range<std::remove_cvref_t<std::remove_pointer_t<T>>>::value;



std::vector<rclcpp::Parameter> declare_parameters(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
        parameter_interface,
    std::vector<rcl_interfaces::msg::Parameter> default_values,
    std::string base_name = "");

std::vector<rclcpp::Parameter> declare_parameters(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
        parameter_interface,
    std::vector<rcl_interfaces::msg::ParameterDescriptor> descriptors,
    std::string base_name = "");

template <typename T>
concept is_param = std::is_same_v<T, int> || std::is_same_v<T, double> ||
                   std::is_same_v<T, bool> || std::is_same_v<T, std::string> ||
                   std::is_same_v<T, std::vector<int>> ||
                   std::is_same_v<T, std::vector<double>> ||
                   std::is_same_v<T, std::vector<std::string>> ||
                   std::is_same_v<T, std::vector<bool>> ||
                   std::is_same_v<T, std::vector<uint8_t>> ||
                   std::is_same_v<T, std::vector<std::string>> ||
                   std::is_same_v<T, std::vector<std::string>>;

template <typename T>
concept has_reflection_type_defined = requires { typename T::ReflectionType; };

template <typename T>
concept has_type_defined = requires { typename T::Type; };

template <typename T>
concept is_convertible_to_param =
    is_param<T> || (has_type_defined<T> && is_param<typename T::Type>);

template <typename T>
concept has_enum_type = has_type_defined<T> && std::is_enum_v<typename T::Type>;

template <typename T> auto get_param_typeinfo() {
  if constexpr (has_reflection_type_defined<T>) {
    return std::type_identity<typename T::ReflectionType>{};
  } else if constexpr (has_type_defined<T>) {
    return std::type_identity<typename T::Type>{};
  } else {
    return std::type_identity<T>{};
  }
};

template <typename EnumType> std::string print_enum_names() {
  std::stringstream ss;
  ss << "{";
  rfl::get_enumerators<EnumType>().apply(
      [&ss](const auto &field) { ss << field.name() << ","; });
  ss << "}";
  return ss.str();
}

template <typename T>
T declare_params(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
                     parameter_interface,
                 std::string base_name = "") {
  T retval;
  const auto view = rfl::to_view(retval);
  view.apply([&parameter_interface,
              &base_name]<typename Field>(const Field &field) {
    using original_type = std::remove_pointer_t<typename Field::Type>;
    using field_type =
        decltype(get_param_typeinfo<
                 std::remove_pointer_t<typename Field::Type>>())::type;

    std::string name;
    if (base_name != "") {
      if (base_name.back() == '.') {
        name = base_name;

      } else {
        name = (base_name + '.');
      }
    }
    name.append(Field::name());

    if constexpr (is_param<field_type>) {
      // field.value() =
      // parameter_interface->declare_parameter<field_type>(name);
      try {

        rclcpp::ParameterValue value{field_type{}};
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.type = value.get_type();
        if constexpr (rfl::internal::is_validator_v<original_type>){
            if constexpr (is_ros_range_v<typename original_type::ValidationType>){

                if constexpr (std::is_same_v<typename original_type::ReflectionType,double>){
                    descriptor.floating_point_range.resize(1);
                    rcl_interfaces::msg::FloatingPointRange range;
                    range.from_value = original_type::ValidationType::minimum;
                    range.to_value = original_type::ValidationType::maximum;
                    range.step = original_type::ValidationType::step;
                    descriptor.floating_point_range[0]= range;
                }else{
                    descriptor.integer_range.resize(1);
                    rcl_interfaces::msg::IntegerRange range;
                    range.from_value = original_type::ValidationType::minimum;
                    range.to_value = original_type::ValidationType::maximum;
                    range.step = original_type::ValidationType::step;
                    descriptor.integer_range[0]= range;
                }

            }

        }
        *field.value() =
            parameter_interface->declare_parameter(name, value.get_type(),descriptor)
                .get<field_type>();
      } catch (const rclcpp::ParameterTypeException &) {
        throw rclcpp::exceptions::
            UninitializedStaticallyTypedParameterException(name);
      }
    } else if constexpr (std::is_enum_v<field_type>) {

      auto value = parameter_interface->declare_parameter(
          name, rclcpp::ParameterType::PARAMETER_STRING);
      auto result = rfl::string_to_enum<field_type>(value.get<std::string>());
      if (result) {
        *field.value() = *result;
      } else {
        throw rclcpp::exceptions::InvalidParameterValueException(
            " Parameter " + name + " is an enum but  " + name +
            " does not match any enum option, options are " +
            print_enum_names<field_type>());
      }
    } else {
      *field.value() = declare_params<field_type>(parameter_interface, name);
    }
  });
  return retval;
}

template <typename T>
T get_params(rclcpp::node_interfaces::NodeParametersInterface::SharedPtr
                 parameter_interface,
             std::string base_name = "") {
  T retval;
  const auto view = rfl::to_view(retval);
  view.apply([&parameter_interface,
              &base_name]<typename Field>(const Field &field) {
    using field_type =
        decltype(get_param_typeinfo<
                 std::remove_pointer_t<typename Field::Type>>())::type;

    std::string name;
    if (base_name != "") {
      if (base_name.back() == '.') {
        name = base_name;

      } else {
        name = (base_name + '.');
      }
    }
    name.append(Field::name());

    if constexpr (is_convertible_to_param<field_type>) {
      // field.value() =
      // parameter_interface->declare_parameter<field_type>(name);
      try {
        rclcpp::ParameterValue value{field_type{}};
        *field.value() =
            parameter_interface->get_parameter(name).get_value<field_type>();
      } catch (const rclcpp::ParameterTypeException &) {
        throw rclcpp::exceptions::
            UninitializedStaticallyTypedParameterException(name);
      }
    } else if constexpr (std::is_enum_v<field_type>) {
      auto value = rfl::string_to_enum<field_type>(
          parameter_interface->get_parameter(name).as_string());

      if (value) {
        *field.value() = *value;
      } else {
        throw rclcpp::exceptions::InvalidParameterValueException(
            " Parameter " + name + " is an enum but  " + name +
            " does not match any enum option, options are " +
            print_enum_names<field_type>());
      }
    } else {
      *field.value() = get_params<field_type>(parameter_interface, name);
    }
  });
  return retval;
}

template <typename Field>
void process_parameter_change(const Field &field,
                              const rclcpp::Parameter &change,
                              std::string base_name = "") {
  using field_type =
      decltype(get_param_typeinfo<
               std::remove_pointer_t<typename Field::Type>>())::type;
  std::string name;
  if (base_name != "") {
    if (base_name.back() == '.') {
      name = base_name;

    } else {
      name = (base_name + '.');
    }
  }
  name.append(Field::name());

  if constexpr (is_convertible_to_param<field_type>) {

    if (change.get_name() == name) {
      rclcpp::ParameterValue value{field_type{}};
      if (change.get_type() == value.get_type()) {
        *field.value() = change.get_parameter_value().get<field_type>();
      } else {
        throw rclcpp::ParameterTypeException(value.get_type(),
                                             change.get_type());
      }
    }
  } else if constexpr (std::is_enum_v<field_type>) {

    if (change.get_name() == name) {
      auto value = rfl::string_to_enum<field_type>(change.as_string());
      if (value) {
        *field.value() = *value;
      } else {
        throw rclcpp::exceptions::InvalidParameterValueException(
            " Parameter " + name + " is an enum but  " + name +
            " does not match any enum option, options are " +
            print_enum_names<field_type>());
      }
    }

  } else {
    const auto view = rfl::to_view(*field.value());
    view.apply([&change, &name]<typename F>(const F &field) {
      process_parameter_change(field, change, name);
    });
  }
}

template <typename T>
rcl_interfaces::msg::SetParametersResult validate_param_changes(
    T prev_value, const std::vector<rclcpp::Parameter> &parameter_changes) {
  rcl_interfaces::msg::SetParametersResult retval;
  const auto view = rfl::to_view(prev_value);
  try {
    for (auto &change : parameter_changes) {
      view.apply([&change]<typename Field>(const Field &field) {
        process_parameter_change(field, change);
      });
    }

  } catch (std::exception &e) {
    retval.successful = false;
    retval.reason = e.what();
    return retval;
  }

  retval.successful = true;
  return retval;
}
} // namespace amtc
