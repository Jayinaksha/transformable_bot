cmake_minimum_required(VERSION 3.8)
project(transformable_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-common5 REQUIRED)

# Include directories
include_directories(include)

# Build ServoY Plugin
add_library(servo_y_plugin SHARED src/servo_y_plugin.cpp)
target_link_libraries(servo_y_plugin
  ${gz-sim8_LIBRARIES}
  ${gz-plugin2_LIBRARIES}
  ${gz-common5_LIBRARIES}
)
ament_target_dependencies(servo_y_plugin
  rclcpp
  std_msgs
)

# Build ServoX Plugin
add_library(servo_x_plugin SHARED src/servo_x_plugin.cpp)
target_link_libraries(servo_x_plugin
  ${gz-sim8_LIBRARIES}
  ${gz-plugin2_LIBRARIES}
  ${gz-common5_LIBRARIES}
)
ament_target_dependencies(servo_x_plugin
  rclcpp
  std_msgs
)

# Build Motor Plugin
add_library(motor_plugin SHARED src/motor_plugin.cpp)
target_link_libraries(motor_plugin
  ${gz-sim8_LIBRARIES}
  ${gz-plugin2_LIBRARIES}
  ${gz-common5_LIBRARIES}
)
ament_target_dependencies(motor_plugin
  rclcpp
  std_msgs
)

# Replace the old mode_switch_plugin section with this:
add_library(dynamic_joint_mode_switch_plugin SHARED src/dynamic_joint_mode_switch_plugin.cpp)
target_link_libraries(dynamic_joint_mode_switch_plugin
  ${gz-sim8_LIBRARIES}
  ${gz-plugin2_LIBRARIES}
  ${gz-common5_LIBRARIES}
)
ament_target_dependencies(dynamic_joint_mode_switch_plugin
  rclcpp
  std_msgs
  std_srvs
)

# Update install section:
install(TARGETS
  servo_y_plugin
  servo_x_plugin
  motor_plugin
  dynamic_joint_mode_switch_plugin
  DESTINATION lib/${PROJECT_NAME}
)# 

# Install other directories
install(DIRECTORY
  launch
  urdf
  worlds
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()



ament_package()
