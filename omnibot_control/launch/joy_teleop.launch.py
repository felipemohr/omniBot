import launch
from launch_ros.actions import Node

def generate_launch_description():

  joy_node = Node(
    package='joy',
    executable='joy_node',
    name='joy_node',
    output='both',
    parameters=[{
      'deadzone': 0.05,
      'autorepeat_rate': 1.0,
      'sticky_buttons': False,
      'coalesce_interval_ms': 1
    }]
  )

  joy_teleop = Node(
    package='omnibot_control',
    executable='joy_teleop',
    name='joy_teleop',
    parameters=[
      {"reverse_button": 10},
      {"stop_button1":   4},
      {"stop_button2":   5},
      {"turbo_button":   2},
      {"slow_button":    3},
      {"axis_linear_x":  1},
      {"axis_linear_y":  0},
      {"axis_angular":   3},
      {"scale_linear":  1.0},
      {"scale_angular": 1.0}
    ]
  )


  return launch.LaunchDescription([
    joy_node,
    joy_teleop
  ])
