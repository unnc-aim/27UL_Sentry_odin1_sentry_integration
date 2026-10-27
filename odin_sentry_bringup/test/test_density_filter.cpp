#include <cstring>
#include <iostream>
#include <random>
#include "odin_cloud_filter.hpp"
#include "pcl/kdtree/kdtree_flann.h"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("density_filter_test", rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("near_outlier_range", 3.0),
    rclcpp::Parameter("near_outlier_radius", 0.04),
    rclcpp::Parameter("near_outlier_min_neighbors", 20),
    rclcpp::Parameter("near_outlier_scale_by_range", true)}));
  OdinCloudFilter filter(*node);
  int result = 0;
  for (int scenario = 0; scenario < 4; ++scenario) {
    auto points = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    if (scenario < 2) {
      const float distance = scenario ? 1.f : .18f;
      for (int x = 0; x < 128; ++x) {
        for (int y = 0; y < 96; ++y) {
          points->push_back({distance, (x - 64) * distance / 128, (y - 48) * distance / 128});
        }
      }
    } else if (scenario == 2) {
      for (int i = 0; i < 400; ++i) {points->push_back({.25f, 0.f, 0.f});}
      for (float x : {.29f, .21000001f, .28999999f, 3.f, 4.f}) {points->push_back({x, 0.f, 0.f});}
    } else {
      std::mt19937 random(42);
      std::uniform_real_distribution<float> coordinate(-1.f, 1.f);
      for (int i = 0; i < 5000; ++i) {points->push_back({coordinate(random), coordinate(random), coordinate(random)});}
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.height = 1;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(points->size());
    for (size_t i = 0; i < points->size(); ++i) {
      std::memcpy(cloud.data.data() + i * cloud.point_step, (*points)[i].data, 12);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> original(false);
    original.setInputCloud(points);
    std::vector<int> indices;
    std::vector<float> distances;
    std::vector<uint8_t> expected;
    for (size_t i = 0; i < points->size(); ++i) {
      const auto & p = (*points)[i];
      const double r2 = tf2::Vector3(p.x, p.y, p.z).length2();
      const int count = static_cast<int>(std::min(double(points->size()), std::ceil(20 / std::max(r2, 1e-12)))) + 1;
      if (r2 >= 9 || original.radiusSearch(p, .04, indices, distances, count) >= count) {
        expected.insert(expected.end(), cloud.data.begin() + i * cloud.point_step,
          cloud.data.begin() + (i + 1) * cloud.point_step);
      }
    }
    filter.apply(cloud, tf2::Vector3(0, 0, 0));
    if (cloud.data != expected) {
      std::cerr << "Density selection differs in scenario " << scenario << std::endl;
      result = 1;
      break;
    }
  }
  if (!result) {std::cout << "Dense walls, duplicate points, radius boundary and sparse geometry match original selection" << std::endl;}
  rclcpp::shutdown();
  return result;
}
