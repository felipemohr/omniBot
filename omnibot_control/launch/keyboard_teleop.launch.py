import launch
from launch_ros.actions import Node

def generate_launch_description():

  keyboard_node = Node(
    package='keyboard_interface',
    executable='keyboard_node.py',
    name='keyboard_node',
    output='both',
    parameters=[{
      'keys_publish_rate': 10.0,
      'sticky_buttons': False,
    }]
  )

  keyboard_teleop = Node(
    package='omnibot_control',
    executable='keyboard_teleop',
    name='keyboard_teleop',
    parameters=[
      {"move_forward_key":             "W"},
      {"move_backward_key":            "S"},
      {"move_left_key":                "A"},
      {"move_right_key":               "D"},
      {"rotate_clockwise_key":         "E"},
      {"rotate_counter_clockwise_key": "Q"},
      {"turbo_key":            "KEY.SHIFT"},
      {"slow_key":               "KEY.ALT"},
      {"increase_linear_key":     "KEY.UP"},
      {"decrease_linear_key":   "KEY.DOWN"},
      {"increase_angular_key": "KEY.RIGHT"},
      {"decrease_angular_key":  "KEY.LEFT"},
      {"scale_linear":                 1.0},
      {"scale_angular":                2.0}
    ]
  )


  return launch.LaunchDescription([
    keyboard_node,
    keyboard_teleop
  ])
