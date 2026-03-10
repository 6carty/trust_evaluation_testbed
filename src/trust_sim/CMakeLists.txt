cmake_minimum_required(VERSION 3.8)
project(trust_sim)

 
# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(pluginlib REQUIRED)

# Add the plugin directory
add_library(actor_plugin SHARED ~/ros_ws/src/trust_sim/plugins/actor_plugin.cpp)
 

# Link dependencies
ament_target_dependencies(actor_plugin
  rclcpp
  gazebo_dev
  gazebo_msgs
  pluginlib
)

install(
    PROGRAMS
        ~/ros_ws/src/trust_sim/scripts/reliability_service.py
        ~/ros_ws/src/trust_sim/scripts/goal_setting_service.py
        ~/ros_ws/src/trust_sim/scripts/comfort_service.py
        ~/ros_ws/src/trust_sim/scripts/goal_monitor.py
        ~/ros_ws/src/trust_sim/scripts/safety_service.py
        ~/ros_ws/src/trust_sim/scripts/trust_service.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install the plugin
install(TARGETS actor_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Install plugin.xml
install(FILES ~/ros_ws/src/trust_sim/plugins/plugin.xml
  DESTINATION share/${PROJECT_NAME})


# Export dependencies
ament_export_dependencies(rclcpp gazebo_dev gazebo_msgs pluginlib)
ament_export_libraries(actor_plugin)

ament_package()
