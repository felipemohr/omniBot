#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "keyboard_teleop/KeyboardTeleop.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

KeyboardTeleop::KeyboardTeleop() : Node("keyboard_teleop")
{
  RCLCPP_INFO(this->get_logger(), "Keyboard Teleop node initialized");
  
  _vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  _keys_subscriber = this->create_subscription<keyboard_interface::msg::Keys>("/keys", 10,
                           std::bind(&KeyboardTeleop::keysCallback, this, _1));
  _key_event_subscriber = this->create_subscription<keyboard_interface::msg::KeyEvent>("/key_event", 10,
                           std::bind(&KeyboardTeleop::keyEventCallback, this, _1));

  _timer = this->create_wall_timer( 50ms, std::bind(&KeyboardTeleop::publishVelocity, this) );

  _move_forward             = this->declare_parameter<std::string>("move_forward_key",             "W");
  _move_backward            = this->declare_parameter<std::string>("move_backward_key",            "S");
  _move_left                = this->declare_parameter<std::string>("move_left_key",                "A");
  _move_right               = this->declare_parameter<std::string>("move_right_key",               "D");
  _rotate_clockwise         = this->declare_parameter<std::string>("rotate_clockwise_key",         "E");
  _rotate_counter_clockwise = this->declare_parameter<std::string>("rotate_counter_clockwise_key", "Q");
  _turbo                    = this->declare_parameter<std::string>("turbo_key",                    "KEY.SHIFT");
  _slow                     = this->declare_parameter<std::string>("slow_key",                     "KEY.ALT");
  _increase_linear          = this->declare_parameter<std::string>("increase_linear_key",          "KEY.UP");
  _decrease_linear          = this->declare_parameter<std::string>("decrease_linear_key",          "KEY.DOWN");
  _increase_angular         = this->declare_parameter<std::string>("increase_angular_key",         "KEY.RIGHT");
  _decrease_angular         = this->declare_parameter<std::string>("decrease_angular_key",         "KEY.LEFT");
  _scale_linear             = this->declare_parameter<double>("scale_linear",  1.0);
  _scale_angular            = this->declare_parameter<double>("scale_angular", 1.0);

  _k_lin = 0.5;
  _k_ang = 0.5;

}

KeyboardTeleop::~KeyboardTeleop()
{
}


void KeyboardTeleop::keysCallback(const keyboard_interface::msg::Keys::SharedPtr msg)
{
  double vel_lin_x = 0.0;
  double vel_lin_y = 0.0;
  double vel_ang_z = 0.0;
  bool turbo_pressed = false;
  bool slow_pressed = false;
  
  for(auto key : msg->pressed_keys)
  {
    if (key.c_str() == _move_forward)                  vel_lin_x += _scale_linear;
    else if (key.c_str() == _move_backward)            vel_lin_x -= _scale_linear;
    else if (key.c_str() == _move_left)                vel_lin_y += _scale_linear;
    else if (key.c_str() == _move_right)               vel_lin_y -= _scale_linear;
    else if (key.c_str() == _rotate_clockwise)         vel_ang_z -= _scale_angular;
    else if (key.c_str() == _rotate_counter_clockwise) vel_ang_z += _scale_angular;
    else if (key.c_str() == _turbo)                    turbo_pressed = true;
    else if (key.c_str() == _slow)                     slow_pressed  = true;
  }

  if (turbo_pressed && !slow_pressed)
  {
    _twist.linear.x  = vel_lin_x;
    _twist.linear.y  = vel_lin_y;
    _twist.angular.z = vel_ang_z;
  }

  else if (slow_pressed && !turbo_pressed)
  {
    _twist.linear.x  = 0.25*vel_lin_x;
    _twist.linear.y  = 0.25*vel_lin_y;
    _twist.angular.z = 0.25*vel_ang_z;
  }

  else
  {
    _twist.linear.x  = _k_lin*vel_lin_x;
    _twist.linear.y  = _k_lin*vel_lin_y;
    _twist.angular.z = _k_ang*vel_ang_z;
  }

}

void KeyboardTeleop::keyEventCallback(const keyboard_interface::msg::KeyEvent::SharedPtr msg)
{
  bool print_info = true;

  if (msg->key.c_str() == _increase_linear && !msg->event)
    _k_lin = std::min(_scale_linear, 1.1*_k_lin);
  else if (msg->key.c_str() == _decrease_linear && !msg->event)
    _k_lin = std::max(0.1*_scale_linear, 0.9*_k_lin);
  else if (msg->key.c_str() == _increase_angular && !msg->event)
    _k_ang = std::min(_scale_angular, 1.1*_k_ang);
  else if (msg->key.c_str() == _decrease_angular && !msg->event)
    _k_ang = std::max(0.1*_scale_angular, 0.9*_k_ang);
  else
    print_info = false;

  if (print_info)
    RCLCPP_INFO(this->get_logger(), "Set linear velocity to %.2f and angular velocity to %.2f.", _k_lin, _k_ang);
}

void KeyboardTeleop::publishVelocity()
{
  _vel_publisher->publish(_twist);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  return 0;
}
