#pragma once

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <vector>

#include "flann/algorithms/kdtree_single_index.h"
#include "flann/algorithms/dist.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Vector3.h"

class OdinCloudFilter
{
  // The filter needs a count predicate, rather than a sorted nearest-point list.
  class SupportCount : public flann::ResultSet<float>
  {
  public:
    SupportCount(float radius_squared, int required)
    : radius_squared_(radius_squared), required_(required) {}
    bool full() const override { return true; }
    void addPoint(float distance, size_t) override
    {
      if (count < required_ && distance < radius_squared_) {
        ++count;
      }
    }
    float worstDist() const override
    {
      // Once the predicate is settled, prune even zero-distance branches.
      return count >= required_ ? -1.0f : radius_squared_;
    }
    int count{0};
  private:
    float radius_squared_;
    int required_;
  };
public:
  explicit OdinCloudFilter(rclcpp::Node & node)
  {
    const double radius = node.declare_parameter("self_filter_radius", 0.0);
    if (!std::isfinite(radius) || radius < 0.0) {
      throw std::invalid_argument("Self filter radius must be finite and nonnegative");
    }
    self_filter_radius_squared_ = radius * radius;
    self_filter_box_min_ =
      node.declare_parameter<std::vector<double>>("self_filter_box_min", std::vector<double>{});
    self_filter_box_max_ =
      node.declare_parameter<std::vector<double>>("self_filter_box_max", std::vector<double>{});
    if (
      self_filter_box_min_.size() != self_filter_box_max_.size() ||
      (!self_filter_box_min_.empty() && self_filter_box_min_.size() != 3)) {
      throw std::invalid_argument("Self filter box requires two empty lists or two XYZ bounds");
    }
    for (size_t i = 0; i < self_filter_box_min_.size(); ++i) {
      if (
        !std::isfinite(self_filter_box_min_[i]) || !std::isfinite(self_filter_box_max_[i]) ||
        self_filter_box_min_[i] >= self_filter_box_max_[i]) {
        throw std::invalid_argument("Self filter box bounds must be finite and min < max");
      }
    }
    const double near_range = node.declare_parameter("near_outlier_range", 0.0);
    near_outlier_radius_ = node.declare_parameter("near_outlier_radius", 0.04);
    const auto neighbors = node.declare_parameter<int64_t>("near_outlier_min_neighbors", 5);
    near_outlier_scale_by_range_ = node.declare_parameter("near_outlier_scale_by_range", false);
    if (
      !std::isfinite(near_range) || near_range < 0.0 || !std::isfinite(near_outlier_radius_) ||
      near_outlier_radius_ <= 0.0 || neighbors < 1 || neighbors > 1000) {
      throw std::invalid_argument(
        "Near outlier filter requires finite distances and 1 to 1000 neighbors");
    }
    near_outlier_range_squared_ = near_range * near_range;
    near_outlier_min_neighbors_ = static_cast<int>(neighbors);
  }

  bool keep(const tf2::Vector3 & local) const
  {
    if (local.length2() < self_filter_radius_squared_) {
      return false;
    }
    return self_filter_box_min_.empty() || local.x() < self_filter_box_min_[0] ||
           local.x() > self_filter_box_max_[0] || local.y() < self_filter_box_min_[1] ||
           local.y() > self_filter_box_max_[1] || local.z() < self_filter_box_min_[2] ||
           local.z() > self_filter_box_max_[2];
  }

  void apply(sensor_msgs::msg::PointCloud2 & cloud, const tf2::Vector3 & origin) const
  {
    if (near_outlier_range_squared_ == 0.0 || cloud.width == 0) {
      return;
    }
    auto points = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    points->reserve(cloud.width);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (size_t i = 0; i < cloud.width; ++i, ++x, ++y, ++z) {
      points->push_back(pcl::PointXYZ(*x, *y, *z));
    }
    flann::Matrix<float> data(points->front().data, points->size(), 3, sizeof(pcl::PointXYZ));
    flann::KDTreeSingleIndex<flann::L2_Simple<float>> tree(
      data, flann::KDTreeSingleIndexParams(15));
    tree.buildIndex();
    const flann::SearchParams search(-1, 0.0f, false);
    size_t kept = 0;
    for (size_t i = 0; i < points->size(); ++i) {
      const auto & p = (*points)[i];
      const auto relative = tf2::Vector3(p.x, p.y, p.z) - origin;
      const double distance_squared = relative.length2();
      if (distance_squared < near_outlier_range_squared_) {
        int neighbors = near_outlier_min_neighbors_;
        if (near_outlier_scale_by_range_) {
          // Normalize support to one meter; the cutoff only selects the checked area.
          neighbors = static_cast<int>(std::min(
            static_cast<double>(points->size()),
            std::ceil(neighbors / std::max(distance_squared, 1e-12))));
        }
        // Include the query point; every other point provides support.
        SupportCount support(
          static_cast<float>(near_outlier_radius_ * near_outlier_radius_), neighbors + 1);
        tree.findNeighbors(support, p.data, search);
        if (support.count < neighbors + 1) {
          continue;
        }
      }
      if (kept != i) {
        std::memmove(
          cloud.data.data() + kept * cloud.point_step, cloud.data.data() + i * cloud.point_step,
          cloud.point_step);
      }
      ++kept;
    }
    sensor_msgs::PointCloud2Modifier(cloud).resize(kept);
  }

private:
  double self_filter_radius_squared_;
  std::vector<double> self_filter_box_min_, self_filter_box_max_;
  double near_outlier_range_squared_, near_outlier_radius_;
  int near_outlier_min_neighbors_;
  bool near_outlier_scale_by_range_;
};
