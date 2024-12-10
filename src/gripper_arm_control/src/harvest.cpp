#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point32.hpp"
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
      // Publishers
      servo_publisher_ = this->create_publisher<std_msgs::msg::Int32>("servo", 10);
      xyz_publisher_ = this->create_publisher<geometry_msgs::msg::Point32>("arm_pos", 10);

      // Subscribers
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/chatter", 10, std::bind(&Harvest::topic_callback, this, _1));
      timer_ = this->create_wall_timer(
      2000ms, std::bind(&Harvest::timer_callback, this));
      
    }

  private:
    void timer_callback()
    {
      auto message = geometry_msgs::msg::Point32();
      message.x = 1;
      message.y = 2;
      message.z = 3;
      RCLCPP_INFO(this->get_logger(), "Publishing Point32: x=%.2f, y=%.2f, z=%.2f", 
        message.x, message.y, message.z);
      xyz_publisher_->publish(message);
    }

    // TODO
    // use Booleans instead of strings
    // expand to full functionality
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
      servo_publisher_->publish(message);
    }

    // timers
    rclcpp::TimerBase::SharedPtr timer_;
    
    // publishers
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr servo_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr xyz_publisher_;

    // Subscribers
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