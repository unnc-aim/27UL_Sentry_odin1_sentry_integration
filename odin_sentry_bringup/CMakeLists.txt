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
install(PROGRAMS scripts/record_route.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME}
  PATTERN "window_calibration.yaml" EXCLUDE
  PATTERN "window_preview.rviz" EXCLUDE)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  enable_testing()
  find_package(nav2_costmap_2d REQUIRED)
  add_executable(test_raw_costmap test/test_raw_costmap.cpp)
  target_compile_features(test_raw_costmap PUBLIC cxx_std_17)
  ament_target_dependencies(test_raw_costmap rclcpp nav2_costmap_2d sensor_msgs tf2_ros)
  add_executable(test_density_filter test/test_density_filter.cpp)
  target_compile_features(test_density_filter PUBLIC cxx_std_17)
  target_include_directories(test_density_filter PRIVATE include)
  target_include_directories(test_density_filter SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_density_filter ${PCL_LIBRARIES})
  ament_target_dependencies(test_density_filter rclcpp sensor_msgs tf2)
  add_test(NAME odin_density_filter COMMAND test_density_filter)
  set_tests_properties(odin_density_filter PROPERTIES TIMEOUT 20
    ENVIRONMENT "ROS_DOMAIN_ID=121;ROS_LOCALHOST_ONLY=1")
  add_test(NAME odin_raw_costmap COMMAND test_raw_costmap)
  set_tests_properties(odin_raw_costmap PROPERTIES TIMEOUT 20
    ENVIRONMENT "ROS_DOMAIN_ID=98;ROS_LOCALHOST_ONLY=1")
  add_test(NAME odin_adapter_pipeline
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pipeline.py
      $<TARGET_FILE:odin_nav_adapter>)
  set_tests_properties(odin_adapter_pipeline PROPERTIES TIMEOUT 45)
  add_test(NAME odin_independent_odometry
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_independent_odometry.py
      $<TARGET_FILE:odin_nav_adapter>)
  set_tests_properties(odin_independent_odometry PROPERTIES TIMEOUT 20
    ENVIRONMENT "PYTHONNOUSERSITE=1;OPENBLAS_NUM_THREADS=1;PYTHONDONTWRITEBYTECODE=1;ROS_DOMAIN_ID=142;ROS_LOCALHOST_ONLY=1")
  add_test(NAME odin_shared_alignment
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_shared_alignment.py
      $<TARGET_FILE:odin_nav_adapter>)
  set_tests_properties(odin_shared_alignment PROPERTIES TIMEOUT 15
    ENVIRONMENT "PYTHONNOUSERSITE=1;OPENBLAS_NUM_THREADS=1;ROS_DOMAIN_ID=119;ROS_LOCALHOST_ONLY=1")
  add_test(NAME odin_launch_configuration
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch.py)
  set_tests_properties(odin_launch_configuration PROPERTIES TIMEOUT 40)
  set_tests_properties(odin_adapter_pipeline odin_launch_configuration
    PROPERTIES RESOURCE_LOCK odin_domain_87)
  add_test(NAME odin_global_localization
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_launch.py --global-search-check)
  set_tests_properties(odin_global_localization PROPERTIES TIMEOUT 90
    ENVIRONMENT "PYTHONNOUSERSITE=1;OPENBLAS_NUM_THREADS=1;ROS_DOMAIN_ID=92;ROS_LOCALHOST_ONLY=1")
  add_test(NAME odin_raw_obstacles
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_raw_obstacles.py
      $<TARGET_FILE:odin_raw_obstacles>)
  set_tests_properties(odin_raw_obstacles PROPERTIES TIMEOUT 25)
  add_test(NAME odin_raw_range_10m
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_raw_obstacles.py
      $<TARGET_FILE:odin_raw_obstacles> 10.0)
  set_tests_properties(odin_raw_range_10m PROPERTIES TIMEOUT 25)
  set_tests_properties(odin_raw_obstacles odin_raw_range_10m
    PROPERTIES RESOURCE_LOCK odin_domain_97)
  add_test(NAME odin_stationary_mapping
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_stationary_mapping.py)
  set_tests_properties(odin_stationary_mapping PROPERTIES TIMEOUT 25)
  add_test(NAME odin_route_selection
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_route.py)
  set_tests_properties(odin_route_selection PROPERTIES TIMEOUT 15
    ENVIRONMENT "ROS_DOMAIN_ID=86;ROS_LOCALHOST_ONLY=1")
  add_test(NAME odin_velocity_timeout
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_velocity_timeout.py)
  set_tests_properties(odin_velocity_timeout PROPERTIES TIMEOUT 35
    ENVIRONMENT "ROS_DOMAIN_ID=95;ROS_LOCALHOST_ONLY=1")
endif()

ament_package()
