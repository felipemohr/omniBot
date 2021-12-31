import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
  pkg_share = FindPackageShare(package='omnibot_description').find('omnibot_description')
  default_model_path = os.path.join(pkg_share, 'urdf/omnibot.urdf.xacro')
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/config.rviz')

  pkg_ros_ign_gazebo = FindPackageShare('ros_ign_gazebo').find('ros_ign_gazebo')


  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}],
    output='both'
  )

  ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
            launch_arguments={'ign_args': '-r empty.sdf'}.items(),
  )



  joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher'
  )


  spawn_entity = Node(
    package='ros_ign_gazebo',
    executable='create',
    arguments=['-name', 'omnibot',
               '-topic', 'robot_description'],
    output='screen'
  )

  
  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', LaunchConfiguration('rvizconfig')]
  )


  # Ign - ROS Bridge
  ign_ros_bridge = Node(
    package='ros_ign_bridge',
    executable='parameter_bridge',
    arguments=[
            # Clock (IGN -> ROS2)
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # Joint states (IGN -> ROS2)
            '/world/empty/model/omnibot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            ],
    remappings=[
        ('/world/empty/model/omnibot/joint_state', 'joint_states'),
    ],
    output='screen'
  )

  return launch.LaunchDescription([
    launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                         description='Absolute path to robot urdf file'),
    launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                         description='Absolute path to rviz config file'),
    launch.actions.DeclareLaunchArgument(name='use_sim_time',default_value='true',
                                         description='Use simulation (Gazebo) clock if true'),
    joint_state_publisher_node,
    ignition,
    spawn_entity,
    ign_ros_bridge,
    robot_state_publisher_node,
    rviz_node
  ])
