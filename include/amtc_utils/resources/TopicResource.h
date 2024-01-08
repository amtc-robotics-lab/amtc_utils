#pragma once

#include <rclcpp/rclcpp.hpp> 
#include <memory>
#include <optional>
#include <amtc_utils/resources/Resource.h>

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

  public:
  
  typedef std::shared_ptr< TopicResource<T> > SharedPtr;

  TopicResource(rclcpp::Node *node, const char* t_topicName)
    :    topic_name_(t_topicName), time_out_duration_(1,0), last_data_time_(0,0,node->get_clock()->get_clock_type()), node_(node)
  {
    subscription_  = node->create_subscription<T>(t_topicName, 5 , std::bind(&TopicResource<T>::msg_cb, this, std::placeholders::_1) );
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
  time_out_duration_ = rclcpp::Duration::from_seconds(period_seconds);
}

/**
 * @brief Set the timeout duration object
 * 
 * @param period 
 */
void set_timeout_duration(rclcpp::Duration period){
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
   return node_->now()-last_data_time_;
 }
/**
 * @brief Recieves new data and resets the timeout
 *  This Function should run on same thread as timer_cb
 * 
 * @param msg 
 */
  void msg_cb(const typename T::ConstSharedPtr msg){
    data_ = msg;
    last_data_time_= node_->now();
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

      return data_ && node_->now()-last_data_time_ < time_out_duration_;
    
  }

/**
 * @brief get a shared_ptr to a const msg
 * if data has timed out or not received this will be null!
 * 
 * @return T::ConstSharedPtr 
 */
  typename T::ConstSharedPtr get()
  {
    if (is_available()){
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
