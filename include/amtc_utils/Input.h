/**
 * @file Input.h
 *
 * @author Isao Parra
 *
 * @date Dec 13, 2016 (creation)
 */

#ifndef AMTC_UTILS_INPUT_H
#define AMTC_UTILS_INPUT_H

#include <ros/ros.h>
#include <amtc_common/Debug.h>

#include <string>

namespace amtc
{

class InputBase
{
public:

  InputBase()
  {
    time_out_duration_    = ros::Duration(1.0);
    last_update_time_     = ros::Time::now();
    has_timed_out_        = false;
    valid_data_           = false;
    id_filter_            = false;
    expected_id_          = "";
    auto_clear_timed_out_ = false;
  }

  void set_time_out_time(ros::Duration timeout)
  {
    time_out_duration_ = timeout;
  }

  void set_time_out_time(double timeout)
  {
    set_time_out_time(ros::Duration(timeout));
  }

  void set_expected_publisher(const std::string& publisher)
  {
    id_filter_ = true;
    expected_id_ = publisher;
  }

  bool is_timed_out()
  {
    bool aux = ros::Time::now() - last_update_time_ > time_out_duration_;

    if(aux == true)
    {
      has_timed_out_ = true;
      valid_data_    = false;
      new_data_      = false;
    }

    return aux;
  }

  bool has_timed_out()
  {
    return has_timed_out_;
  }

  bool has_valid_data()
  {
    return valid_data_;
  }

  bool has_new_data()
  {
    return new_data_;
  }

  void clear_timed_out()
  {
    valid_data_       = false;
    has_timed_out_    = false;
    new_data_         = false;
    last_update_time_ = ros::Time::now();
  }

  ros::Time last_message_time()
  {
    return last_update_time_;
  }

  void set_auto_clear_timed_out(bool value)
  {
    auto_clear_timed_out_ = value;
  }

  void msg_cb_base()
  {
    is_timed_out();

    last_update_time_ = ros::Time::now();
    valid_data_       = true;
    new_data_         = true;

    if(auto_clear_timed_out_)
      has_timed_out_ = false;
  }

protected:

  ros::Time last_update_time_;
  ros::Duration time_out_duration_;
  bool has_timed_out_;
  bool valid_data_;
  bool new_data_; /* This flag allows knowing if this message has been read before */
  bool id_filter_; /* This flag indicates wether messages should be filtered according to the expected id */
  std::string expected_id_; /* Used together with the id_filter_ flag to specify an expected publisher */
  bool auto_clear_timed_out_; /* Wether or not timed out flags should recover automatically when a new msg arrives */
};

template<class Msg_Type> class Input : public InputBase
{
public:

  Input() :
    InputBase()
  {
    additional_callback_.clear();
  }

  void set_additional_callback(boost::function<void(const typename Msg_Type::ConstPtr&)> f)
  {
    additional_callback_ = f;
  }

  void msg_cb(const typename Msg_Type::ConstPtr& msg)
  {
    msg_cb_base();

    data_ = msg->data;

    if(additional_callback_)
      additional_callback_(msg);
  }

  void msg_with_publisher_id_cb(const typename Msg_Type::Ptr& msg, const std::string& publisher)
  {
    if(id_filter_ && expected_id_ != publisher)
      return;

    msg_cb_base();

    data_ = msg->data;
  }

  void set_value(const typename Msg_Type::_data_type value)
  {
    is_timed_out();

    last_update_time_ = ros::Time::now();
    data_             = value;
    valid_data_       = true;
  }

  void set_default_value(typename Msg_Type::_data_type value)
  {
    default_data_ = value;
  }

  typename Msg_Type::_data_type& get()
  {
    is_timed_out();
    new_data_ = false;

    if(has_timed_out() == true)
    {
      data_ = default_data_;
    }

    return data_;
  }

private:

  boost::function<void(const typename Msg_Type::ConstPtr&)> additional_callback_;

  typename Msg_Type::_data_type data_;
  typename Msg_Type::_data_type default_data_;
};



template<class Msg_Type> class InputGeneric : public InputBase
{
public:

  InputGeneric() :
    InputBase()
  {
    additional_callback_.clear();
  }

  void set_additional_callback(boost::function<const typename Msg_Type::ConstPtr&>& f)
  {
    additional_callback_ = f;
  }

  void msg_cb(const typename Msg_Type::ConstPtr& msg)
  {
    msg_cb_base();

    data_ = msg;

    if(additional_callback_)
      additional_callback_(msg);
  }

  void msg_with_publisher_id_cb(const typename Msg_Type::Ptr& msg, const std::string& publisher)
  {
    /* Filter out unexpected publishers */
    if(id_filter_ && publisher != expected_id_)
      return;

    msg_cb_base();

    data_ = msg;
  }

  void set_default_data(const Msg_Type& def)
  {
    default_data_ = def;
  }

  const Msg_Type& get()
  {
    new_data_ = false;

    if(!valid_data_)
    {
      if(data_)
      {
        // AMTC_BREAKPOINT;
        ROS_WARN_NODE("Attempting to read timed-out data from %s message. This is highly discouraged", ros::message_traits::DataType<Msg_Type>::value());
        return *data_;
      }
      else
      {
        AMTC_BREAKPOINT;
        ROS_WARN_NODE_THROTTLE(1.0,"Attempting to read %s message when no message has ben received yet. Returning default value", ros::message_traits::DataType<Msg_Type>::value());
        return default_data_;
      }
    }

    return *data_;
  }

protected:

  typename Msg_Type::ConstPtr data_;
  Msg_Type default_data_;
  boost::function<void(const typename Msg_Type::ConstPtr&)> additional_callback_;

};


}

#endif /* AMTC_UTILS_INPUT_H */
