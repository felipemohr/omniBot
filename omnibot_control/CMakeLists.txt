cmake_minimum_required(VERSION 3.5)
project(omnibot_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(keyboard_interface REQUIRED)

add_executable(test_vel_control src/test_vel_control.cpp)
add_executable(joy_teleop src/JoyTeleop.cpp)
add_executable(keyboard_teleop src/KeyboardTeleop.cpp)

ament_target_dependencies(test_vel_control rclcpp geometry_msgs)
ament_target_dependencies(joy_teleop rclcpp geometry_msgs sensor_msgs)
ament_target_dependencies(keyboard_teleop rclcpp geometry_msgs keyboard_interface)

include_directories(include)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS test_vel_control joy_teleop keyboard_teleop
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
