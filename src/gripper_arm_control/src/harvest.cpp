#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Harvest : public rclcpp::Node
{
  public:
    Harvest()
    : Node("harvest"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::Int32>("servo", 10);
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/chatter", 10, std::bind(&Harvest::topic_callback, this, _1));
      // timer_ = this->create_wall_timer(
      // 500ms, std::bind(&Harvest::timer_callback, this));
    }

  private:
    // void timer_callback()
    // {
      // auto message = std_msgs::msg::Int32();
      // message.data = count_++;
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%d' ", message.data);
      // publisher_->publish(message);
    // }
    void topic_callback(const std_msgs::msg::String & msg) const {
      auto message = std_msgs::msg::Int32();
      if(msg.data == "open"){
        RCLCPP_INFO(this->get_logger(), "If I heard: '%s'", msg.data.c_str());
        message.data = 180;
      }
      else if(msg.data == "closed"){
        RCLCPP_INFO(this->get_logger(), "Else if I heard: '%s'", msg.data.c_str());
        message.data = 0;
      }
      else{
        message.data = 90;
      }
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    int count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Harvest>());
  rclcpp::shutdown();
  return 0;
}