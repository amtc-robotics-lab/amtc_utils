#pragma once

#include <rclcpp/rclcpp.hpp> 
#include <memory>
#include <optional>
#include <amtc_utils/resources/Resource.h>

namespace amtc{
class AbstractTopicResource : public Resource
{
  public:
  bool isAvailable()
  {
    return false;
  }

  bool has_valid_data()
  {
    return false;
  }

  bool has_new_data()
  {
    return false;
  }

  bool isNull()
  {
    return false;
  }

  virtual void clear_timed_out() { }
};

class NullTopicResource : public AbstractTopicResource
{
  private:
  NullTopicResource() {}

  public:
  bool isNull()
  {
    return true;
  }

  static NullTopicResource& getInstance()
  {
    static NullTopicResource instance; // Guaranteed to be destroyed.
                       // Instantiated on first use.
    return instance;
  }
  NullTopicResource(NullTopicResource const&) = delete;
  void operator=(NullTopicResource const&) = delete;
};

template<typename T> class TopicResource : public AbstractTopicResource
{

protected:

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<T>::SharedPtr subscription_;

  ros::Time last_update_time_;
  ros::Duration time_out_duration_;
  bool new_data_;
  typename T::ConstPtr data_;
  std:optional<T> default_data_;
  std::string topic_name_;

  public:
  
  typedef std::shared_ptr< TopicResource<T> > Ptr;

  TopicResource(rclcpp::Node::SharedPtr node, const char* t_topicName)
    : node_(node), topic_name_(t_topicName)
  {
    subscription_  = node_->create_subscription<T>(t_topicName, 5 , std::bind(&TopicResource<T>::msg_cb, this, _1) );
    time_out_duration_    = rclcpp::Duration(1.0);
    last_update_time_     = rclcpp::Time();
    new_data_ = false;
  }

  void set_default_value(T def_msg){
    default_data_ = def_msg;
  }


  void set_time_out_time(ros::Duration timeout)
  {
    time_out_duration_ = timeout;
  }


  void msg_cb(const typename T::ConstPtr& msg){
    data_ = msg;
    last_update_time_ = ros::Time::now();
    new_data_         = true;
  }

  inline bool isAvailable(){
    return is_available();
  };

  inline bool is_available()
  {
    bool timed_out = ros::Time::now() - last_update_time_ > time_out_duration_;
    return data_ && !timed_out;
  }



  bool has_new_data()
  {
    return new_data_;
  }

  std::optional<T> get()
  {
    new_data_ = false;

    if(data_)
    {
      if(!is_available())
      {
        ROS_WARN_NODE_THROTTLE(1.0,"Attempting to read %s message on topic %s but data is timed out", ros::message_traits::DataType<T>::value(), topic_name_.c_str());
        return default_data_;
      }
      return *data_;
    }
      
    ROS_WARN_NODE_THROTTLE(5.0, "Reading message on topic %s before any topic is received, returning default",topic_name_.c_str());
    
    return default_data_;
  }



};

}