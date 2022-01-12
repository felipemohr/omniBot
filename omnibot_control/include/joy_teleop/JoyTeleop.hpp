#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyTeleop : public rclcpp::Node
{
public:
  JoyTeleop();
  ~JoyTeleop();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void publishVelocity();

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_publisher;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _joy_subscriber;

  geometry_msgs::msg::Twist _twist;

  bool _rev;
  int _reverse;
  int _stop1, _stop2;
  int _turbo, _slow;
  int _linear_x, _linear_y, _angular;
  double _lin_scale, _ang_scale, _k;

};
