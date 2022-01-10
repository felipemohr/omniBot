#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TestVelPublisher
{
public:
  TestVelPublisher(rclcpp::Node::SharedPtr node) : node_(node)
  {
    publisher_ = this->node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  void publishVelocity(double velLinX, double velLinY, double velAngZ)
  {
    RCLCPP_INFO(this->node_->get_logger(), "Publishing velocity: VelLinX=%.2f  VelLinY=%.2f  VelAngZ=%.2f", velLinX, velLinY, velAngZ);

    auto message = geometry_msgs::msg::Twist();
    message.linear.x  = velLinX;
    message.linear.y  = velLinY;
    message.angular.z = velAngZ;

    size_t count = 0;
    while (count < 20)
    {
      publisher_->publish(message);
      count++;
      std::this_thread::sleep_for(100ms);
    }
  }
  
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Node::SharedPtr node_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("test_vel_control");

  auto vel_publisher = TestVelPublisher(node);

  vel_publisher.publishVelocity( 0.5,  0.0,  0.0);
  vel_publisher.publishVelocity(-0.5,  0.0,  0.0);
  vel_publisher.publishVelocity( 0.0,  0.5,  0.0);
  vel_publisher.publishVelocity( 0.0, -0.5,  0.0);
  vel_publisher.publishVelocity( 0.0,  0.0,  1.0);
  vel_publisher.publishVelocity( 0.0,  0.0, -1.0);
  vel_publisher.publishVelocity( 0.5,  0.5,  0.0);
  vel_publisher.publishVelocity( 0.5, -0.5,  0.0);
  vel_publisher.publishVelocity(-0.5, -0.5,  0.0);
  vel_publisher.publishVelocity(-0.5,  0.5,  0.0);
  vel_publisher.publishVelocity( 0.3, -0.2,  0.6);
  vel_publisher.publishVelocity(-0.4,  0.6, -1.0);
  vel_publisher.publishVelocity( 0.0,  0.0,  0.0);

  rclcpp::shutdown();
  return 0;
}
