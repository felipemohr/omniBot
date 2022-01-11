#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "keyboard_interface/msg/keys.hpp"
#include "keyboard_interface/msg/key_event.hpp"

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop();
  ~KeyboardTeleop();

private:
  void keysCallback(const keyboard_interface::msg::Keys::SharedPtr msg);
  void keyEventCallback(const keyboard_interface::msg::KeyEvent::SharedPtr msg);
  void publishVelocity();

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _vel_publisher;
  rclcpp::Subscription<keyboard_interface::msg::Keys>::SharedPtr _keys_subscriber;
  rclcpp::Subscription<keyboard_interface::msg::KeyEvent>::SharedPtr _key_event_subscriber;

  geometry_msgs::msg::Twist _twist;

  std::string _move_forward, _move_backward;
  std::string _move_left, _move_right;
  std::string _rotate_clockwise, _rotate_counter_clockwise;
  std::string _increase_linear, _decrease_linear;
  std::string _increase_angular, _decrease_angular;
  std::string _turbo, _slow;

  double _scale_linear, _scale_angular;
  double _k_lin, _k_ang;

};
