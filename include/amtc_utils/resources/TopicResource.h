#pragma once

#include <rclcpp/rclcpp.hpp> 
#include <memory>
#include <optional>
#include <amtc_utils/resources/Resource.h>
#include <mutex>

namespace amtc{

template<typename T> class TopicResource : public Resource
{

protected:

  typename rclcpp::Subscription<T>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  rclcpp::Duration time_out_duration_;
  rclcpp::Time last_data_time_;
  typename T::ConstSharedPtr data_;
  typename T::ConstSharedPtr default_data_;
  std::string topic_name_;
  rclcpp::Node * node_;
  std::string access_token_;
  bool check_access_token_ = false;
  std::mutex data_mutex_;

  public:
  
  typedef std::shared_ptr< TopicResource<T> > SharedPtr;

  TopicResource(rclcpp::Node *node, const char* t_topicName)
    :    topic_name_(t_topicName), time_out_duration_(1,0), last_data_time_(0,0,node->get_clock()->get_clock_type()), node_(node)
  {
    subscription_  = node->create_subscription<T>(t_topicName, 5 , std::bind(&TopicResource<T>::msg_cb, this, std::placeholders::_1));
  }


  TopicResource(rclcpp::Node *node, const char* t_topicName, double period_seconds)
    :    topic_name_(t_topicName), time_out_duration_(rclcpp::Duration::from_seconds(period_seconds)), last_data_time_(0,0,node->get_clock()->get_clock_type()), node_(node)
  {
    subscription_  = node->create_subscription<T>(t_topicName, 5 , std::bind(&TopicResource<T>::msg_cb, this, std::placeholders::_1) );
  }

  TopicResource(rclcpp::Node *node, const char* t_topicName, rclcpp::Duration period)
    :    topic_name_(t_topicName), time_out_duration_(period), last_data_time_(0,0,node->get_clock()->get_clock_type()), node_(node)
  {
    subscription_  = node->create_subscription<T>(t_topicName, 5 , std::bind(&TopicResource<T>::msg_cb, this, std::placeholders::_1) );
  }

/**
 * @brief Set the timeout duration object
 * 
 * @param period_seconds 
 */
void set_timeout_duration(double period_seconds){
  std::scoped_lock lock(data_mutex_);
  time_out_duration_ = rclcpp::Duration::from_seconds(period_seconds);
}

/**
 * @brief Set the timeout duration object
 * 
 * @param period 
 */
void set_timeout_duration(rclcpp::Duration period){
    std::scoped_lock lock(data_mutex_);
  time_out_duration_ = period;
}


/**
 * @brief  removes the stored msg when timing out
 * This Function should run on same thread as msg_cb
 * 
 */
  void timer_cb(){
    // std::cout<< "timing out\n";
    if(node_->now()-last_data_time_ > time_out_duration_){
      data_.reset();
    }
  }

/**
 * @brief  returns the time since last data was received
 * 
 * @return rclcpp::Duration 
 */
 rclcpp::Duration time_since_last_data(){
    std::scoped_lock lock(data_mutex_);
   return node_->now()-last_data_time_;
 }
/**
 * @brief Recieves new data and resets the timeout
 *  This Function should run on same thread as timer_cb
 * 
 * @param msg 
 */
  void msg_cb(const typename T::ConstSharedPtr msg){
    std::scoped_lock lock(data_mutex_);
    data_ = msg;
    last_data_time_= node_->now();
//    RCLCPP_INFO(node_->get_logger(), "received data on topic %s", topic_name_.c_str());
  }

/**
 * @brief alias of is_available()
 * 
 * @return true 
 * @return false 
 */
  inline bool isAvailable(){
    return is_available();
  };

/**
 * @brief check if data is available and not timed out 
 * Note that data could timeout between a call to isAvailable and get()
 * 
 * @return true data is received and has not timed out
 * @return false  data not available or not timed out
 */
  inline bool is_available()
  {
    std::scoped_lock lock(data_mutex_);
//    RCLCPP_INFO(node_->get_logger(), "checking availability of topic %s", topic_name_.c_str());
      return data_ && node_->now()-last_data_time_ < time_out_duration_;
    
  }

  /**
   * @brief set the default data to return if data is not available
   *
   * @param msg
   */
  void set_default_data(const typename T::ConstSharedPtr msg){
    default_data_ = msg;
  }

/**
 * @brief get a shared_ptr to a const msg
 * if data has timed out or not received this will be null!
 * 
 * @return T::ConstSharedPtr 
 */
  typename T::ConstSharedPtr get()
  {
    std::scoped_lock lock(data_mutex_);
    if (data_ && node_->now()-last_data_time_ < time_out_duration_ ){
      return data_;
    }
    return default_data_; //usually null
    
  }

  void set_access_token(const std::string &token){
    access_token_ = token;
    rclcpp::SubscriptionOptions sub_options;
    sub_options.content_filter_options.filter_expression = "access_token.data == %0";
    sub_options.content_filter_options.expression_parameters = {
            token
    };
    subscription_  = node_->create_subscription<T>(topic_name_, 5 , std::bind(&TopicResource<T>::msg_cb, this, std::placeholders::_1) );


  }

};

} // namespace amtc
