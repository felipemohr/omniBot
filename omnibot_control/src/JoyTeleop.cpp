#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "joy_teleop/JoyTeleop.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

JoyTeleop::JoyTeleop() : Node("joy_teleop")
{
  RCLCPP_INFO(this->get_logger(), "Joy Teleop node initialized");
  
  _vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  _joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10,
                          std::bind(&JoyTeleop::joyCallback, this, _1));

  _timer = this->create_wall_timer( 50ms, std::bind(&JoyTeleop::publishVelocity, this) );

  this->declare_parameter<int>("reverse_button", 10);
  this->declare_parameter<int>("stop_button1",   4);
  this->declare_parameter<int>("stop_button2",   5);
  this->declare_parameter<int>("turbo_button",   2);
  this->declare_parameter<int>("slow_button",    3);
  this->declare_parameter<int>("axis_linear_x",  1);
  this->declare_parameter<int>("axis_linear_y",  0);
  this->declare_parameter<int>("axis_angular",   3);
  this->declare_parameter<double>("scale_linear",  1.0);
  this->declare_parameter<double>("scale_angular", 1.0);

  this->loadParameters();
}

JoyTeleop::~JoyTeleop()
{
}

void JoyTeleop::loadParameters()
{
  this->get_parameter("reverse_button", _reverse);
  this->get_parameter("stop_button1",   _stop1);
  this->get_parameter("stop_button2",   _stop2);
  this->get_parameter("turbo_button",   _turbo);
  this->get_parameter("slow_button",    _slow);
  this->get_parameter("axis_linear_x",  _linear_x);
  this->get_parameter("axis_linear_y",  _linear_y);
  this->get_parameter("axis_angular",   _angular);
  this->get_parameter("scale_linear",   _lin_scale);
  this->get_parameter("scale_angular",  _ang_scale);

  _rev = true;
}

void JoyTeleop::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if(msg->buttons[_reverse]) _rev = !_rev;
  
  _k = 0.5;
  if(msg->buttons[_turbo]) _k = 1.0;
  if(msg->buttons[_slow]) _k = 0.25;
  if(msg->buttons[_stop1] || msg->buttons[_stop2]) _k = 0;
  
  _twist.linear.x = _k*_lin_scale*msg->axes[_linear_x];
  _twist.linear.y = _k*_lin_scale*msg->axes[_linear_y];

  if(msg->axes[_linear_x] < 0 && _rev) _k = -_k;
  _twist.angular.z = _k*_ang_scale*msg->axes[_angular];

}

void JoyTeleop::publishVelocity()
{
  _vel_publisher->publish(_twist);
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyTeleop>());
  rclcpp::shutdown();
  return 0;
}
