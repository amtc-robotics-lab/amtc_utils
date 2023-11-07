#include <amtc_utils/resources/TopicResource.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>


int N=10;

class ResourceChecker : public rclcpp::Node{

public:
    ResourceChecker():  Node("resource_checker"){

        using namespace std::chrono_literals;
    
        for(int i= 0; i< N; i++){
            std::string topic= "float_in/f";
            topic.append(std::to_string(i));
            float_res_.push_back(std::make_shared<amtc::TopicResource<std_msgs::msg::Float64> >(this, topic.c_str() , 0.0001));
        }
        timer_ = rclcpp::create_timer(this, this->get_clock(), rclcpp::Duration::from_seconds(0.001), std::bind(&ResourceChecker::timer_cb, this)  );

    }

    void timer_cb(){
        for (auto& res:float_res_){
            a_ = res->is_available();
            // std::cout << "  " << a_ ;
        }
        // std::cout << "\n";
    }

    bool a_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<amtc::TopicResource<std_msgs::msg::Float64>::SharedPtr > float_res_;
};



int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ResourceChecker>());

    rclcpp::shutdown();

    return 0;
}


