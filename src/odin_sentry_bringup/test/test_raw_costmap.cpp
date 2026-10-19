#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("raw_costmap_input");
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("global_frame", "odom"),
    rclcpp::Parameter("robot_base_frame", "test_base"),
    rclcpp::Parameter("rolling_window", true),
    rclcpp::Parameter("width", 4),
    rclcpp::Parameter("height", 4),
    rclcpp::Parameter("resolution", 0.05),
    rclcpp::Parameter("update_frequency", 20.0),
    rclcpp::Parameter("plugins", std::vector<std::string>{"static_layer", "intensity_voxel_layer"}),
    rclcpp::Parameter("static_layer.plugin", "nav2_costmap_2d::StaticLayer"),
    rclcpp::Parameter("static_layer.map_topic", "/test_map"),
    rclcpp::Parameter("intensity_voxel_layer.plugin", "pb_nav2_costmap_2d::IntensityVoxelLayer"),
    rclcpp::Parameter("intensity_voxel_layer.origin_z", 0.0),
    rclcpp::Parameter("intensity_voxel_layer.mark_threshold", 0),
    rclcpp::Parameter("intensity_voxel_layer.observation_sources", "raw"),
    rclcpp::Parameter("intensity_voxel_layer.raw.topic", "/odin_obstacles/terrain_map"),
    rclcpp::Parameter("intensity_voxel_layer.raw.data_type", "PointCloud2"),
    rclcpp::Parameter("intensity_voxel_layer.raw.sensor_frame", "odin_lidar"),
    rclcpp::Parameter("intensity_voxel_layer.raw.expected_update_rate", 0.5),
    rclcpp::Parameter("intensity_voxel_layer.raw.observation_persistence", 0.0),
    rclcpp::Parameter("intensity_voxel_layer.raw.obstacle_max_range", 3.0),
    rclcpp::Parameter("intensity_voxel_layer.raw.max_obstacle_height", 2.0),
  });
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(costmap->get_node_base_interface());
  std::thread thread([&]() { executor.spin(); });
  auto pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("/odin_obstacles/terrain_map", 5);
  tf2_ros::StaticTransformBroadcaster broadcaster(node);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = node->now();
  tf.header.frame_id = "odom";
  tf.child_frame_id = "test_base";
  tf.transform.rotation.w = 1.0;
  auto lidar = tf;
  lidar.header.frame_id = "test_base";
  lidar.child_frame_id = "odin_lidar";
  lidar.transform.translation.z = 0.43;
  broadcaster.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{tf, lidar});
  auto map_pub = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/test_map", rclcpp::QoS(1).transient_local());
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "odom";
  grid.header.stamp = node->now();
  grid.info.width = 80;
  grid.info.height = 80;
  grid.info.resolution = 0.05;
  grid.info.origin.position.x = -2.0;
  grid.info.origin.position.y = -2.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(6400, 0);
  map_pub->publish(grid);
  int result = 0;
  try {
    costmap->configure();
    costmap->activate();
    auto send = [&](bool obstacle) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id = "odom";
      cloud.header.stamp = node->now();
      cloud.height = 1;
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(4, "x", 1, 7, "y", 1, 7, "z", 1, 7, "intensity", 1, 7);
      modifier.resize(2);
      sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"),
        i(cloud, "intensity");
      *x = 1.025;
      *y = 0.025;
      *z = 0.2;
      *i = obstacle ? 0.3 : 0.0;
      ++x;
      ++y;
      ++z;
      ++i;
      *x = 1.025;
      *y = 0.325;
      *z = 0.0;
      *i = 0.0;
      pub->publish(cloud);
    };
    auto cell = [&](double y) {
      auto map = costmap->getCostmap();
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      unsigned int mx, my;
      if (!map->worldToMap(1.025, y, mx, my)) {
        throw std::runtime_error("Point outside test map");
      }
      return map->getCost(mx, my);
    };
    auto wait = [&](auto condition, bool publish, bool obstacle) {
      const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
      do {
        if (publish) {
          send(obstacle);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (condition()) {
          return;
        }
      } while (std::chrono::steady_clock::now() < until);
      throw std::runtime_error("Costmap condition timed out");
    };
    wait(
      [&]() { return cell(0.025) == nav2_costmap_2d::LETHAL_OBSTACLE && costmap->isCurrent(); },
      true, true);
    if (cell(0.325) == nav2_costmap_2d::LETHAL_OBSTACLE) {
      throw std::runtime_error("Ground marked occupied");
    }
    wait([&]() { return !costmap->isCurrent(); }, false, false);
    wait(
      [&]() { return cell(0.025) != nav2_costmap_2d::LETHAL_OBSTACLE && costmap->isCurrent(); },
      true, false);
    std::cout << "PASS: native obstacle height, ground rejection, source expiry and recovery\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  costmap->deactivate();
  costmap->cleanup();
  executor.cancel();
  thread.join();
  rclcpp::shutdown();
  return result;
}
