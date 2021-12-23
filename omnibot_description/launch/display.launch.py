import launch
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
  pkg_share = FindPackageShare(package='omnibot_description').find('omnibot_description')
  default_model_path = os.path.join(pkg_share, 'urdf/omnibot.urdf')
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
  )
  joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
  )
  joint_state_publisher_gui_node = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='joint_state_publisher_gui',
    condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
  )
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', LaunchConfiguration('rvizconfig')],
  )

  return launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                         description='Flag to enable joint_state_publisher_gui'),
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                         description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                         description='Absolute path to rviz config file'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time',default_value='true',
                                         description='Use simulation (Gazebo) clock if true'),
    joint_state_publisher_gui_node,
    joint_state_publisher_node,
    robot_state_publisher_node,
    rviz_node
  ])
