cmake_minimum_required(VERSION 3.8)
project(odin_sentry_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS kdtree)

add_executable(odin_nav_adapter src/odin_nav_adapter.cpp)
add_executable(odin_raw_obstacles src/odin_raw_obstacles.cpp)
foreach(target odin_nav_adapter odin_raw_obstacles)
  target_compile_features(${target} PUBLIC cxx_std_17)
  target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  target_include_directories(${target} PRIVATE include)
  target_include_directories(${target} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(${target} ${PCL_LIBRARIES})
  ament_target_dependencies(${target}
    rclcpp message_filters nav_msgs sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
endforeach()
install(TARGETS odin_nav_adapter odin_raw_obstacles DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  find_package(nav2_costmap_2d REQUIRED)
  add_executable(test_raw_costmap test/test_raw_costmap.cpp)
  target_compile_features(test_raw_costmap PUBLIC cxx_std_17)
  ament_target_dependencies(test_raw_costmap rclcpp nav2_costmap_2d sensor_msgs tf2_ros)
  add_test(NAME odin_raw_costmap COMMAND test_raw_costmap)
  set_tests_properties(odin_raw_costmap PROPERTIES TIMEOUT 20
    ENVIRONMENT "ROS_DOMAIN_ID=98;ROS_LOCALHOST_ONLY=1")
  add_test(NAME odin_adapter_pipeline
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pipeline.py
      $<TARGET_FILE:odin_nav_adapter>)
  set_tests_properties(odin_adapter_pipeline PROPERTIES TIMEOUT 45)
  add_test(NAME odin_launch_configuration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch.py)
  set_tests_properties(odin_launch_configuration PROPERTIES TIMEOUT 40)
  add_test(NAME odin_raw_obstacles
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_raw_obstacles.py
      $<TARGET_FILE:odin_raw_obstacles>)
  set_tests_properties(odin_raw_obstacles PROPERTIES TIMEOUT 25)
  add_test(NAME odin_stationary_mapping
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_stationary_mapping.py)
  set_tests_properties(odin_stationary_mapping PROPERTIES TIMEOUT 25)
endif()

ament_package()
