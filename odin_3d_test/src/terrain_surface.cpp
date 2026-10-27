#include "surface.hpp"
#include <deque>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
class Terrain : public rclcpp::Node {
  odin3d::Settings s;
  double size, max_age;
  rclcpp::Time last_surface_stamp{0, 0, RCL_ROS_TIME};
  bool last_valid_surface{false};
  int cells;
  tf2_ros::Buffer tf;
  tf2_ros::TransformListener listener;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready;
  rclcpp::TimerBase::SharedPtr timer;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending;

public:
  Terrain() : Node("terrain_surface"), tf(get_clock()), listener(tf) {
    s.flat_ground = declare_parameter("flat_ground", true);
    s.height = declare_parameter("height", 0.50);
    s.clearance = declare_parameter("clearance", 0.05);
    s.slope = declare_parameter("slope_degrees", 30.0);
    s.resolution = declare_parameter("resolution", 0.05);
    s.step = declare_parameter("step_height", 0.03);
    s.obstacle_height = declare_parameter("obstacle_height", 0.10);
    s.min_points = declare_parameter("min_points", 3);
    s.obstacle_support = declare_parameter("obstacle_support", 3);
    s.clearance_support = declare_parameter("clearance_support", 6);
    s.validate();
    size = declare_parameter("window", 6.0);
    max_age = declare_parameter("max_age", 0.5);
    if (!std::isfinite(size) || size < 1 || size > 20 ||
        !std::isfinite(max_age) || max_age <= 0)
      throw std::invalid_argument("Invalid terrain window or age");
    cells = std::ceil(size / s.resolution);
    pub = create_publisher<nav_msgs::msg::OccupancyGrid>("/odin_3d/surface", 1);
    ready = create_publisher<std_msgs::msg::Bool>("/odin_3d/terrain_ready", 1);
    sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/odin_obstacles/registered_scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          pending.push_back(msg);
          if (pending.size() > 5)
            pending.pop_front();
        });
    const double rate = declare_parameter("update_rate", 10.0);
    if (!std::isfinite(rate) || rate < 1.0 || rate > 30.0)
      throw std::invalid_argument("Terrain update rate requires 1 to 30 Hz");
    timer = create_wall_timer(std::chrono::milliseconds(std::lround(1000. / rate)),
                              [this]() { tick(); });
  }
  void publishReady() {
    std_msgs::msg::Bool status;
    const double age = (now() - last_surface_stamp).seconds();
    status.data = last_valid_surface && age >= -0.05 && age <= max_age;
    ready->publish(status);
  }
  void tick() {
    std_msgs::msg::Bool ok;
    ok.data = false;
    // Raw exposure ends can precede arrival of body TF for that timestamp.
    // Select the newest fresh frame whose measurement-time transform exists.
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg;
    for (auto it = pending.rbegin(); it != pending.rend(); ++it) {
      const double age = (now() - rclcpp::Time((*it)->header.stamp)).seconds();
      if ((*it)->header.frame_id == "odom" && age >= -.05 && age <= max_age &&
          tf.canTransform("odom", "base_footprint",
                          rclcpp::Time((*it)->header.stamp))) {
        msg = *it;
        break;
      }
    }
    while (!pending.empty()) {
      const auto stamp = rclcpp::Time(pending.front()->header.stamp);
      if ((now() - stamp).seconds() > max_age ||
          (msg && stamp <= rclcpp::Time(msg->header.stamp)))
        pending.pop_front();
      else
        break;
    }
    if (!msg) {
      publishReady();
      return;
    }
    try {
      auto base = tf.lookupTransform("odom", "base_footprint",
                                     rclcpp::Time(msg->header.stamp));
      auto p = base.transform.translation;
      pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::fromROSMsg(*msg, cloud);
      auto q = base.transform.rotation;
      Eigen::Vector3d normal =
          Eigen::Quaterniond(q.w, q.x, q.y, q.z) * Eigen::Vector3d::UnitZ();
      if (!normal.allFinite() || normal.z() < 0.5) {
        ready->publish(ok);
        return;
      }
      auto grid = odin3d::classify(
          cloud, std::floor((p.x - size / 2) / s.resolution) * s.resolution,
          std::floor((p.y - size / 2) / s.resolution) * s.resolution,
          cells, cells, s, p.x, p.y, p.z,
          true, -normal.x() / normal.z(), -normal.y() / normal.z());
      if ((now() - rclcpp::Time(msg->header.stamp)).seconds() > max_age) {
        ready->publish(ok);
        return;
      }
      nav_msgs::msg::OccupancyGrid out;
      out.header = msg->header;
      out.info.resolution = s.resolution;
      out.info.width = cells;
      out.info.height = cells;
      out.info.origin.position.x = grid.x;
      out.info.origin.position.y = grid.y;
      out.info.origin.orientation.w = 1;
      out.data = grid.cost;
      pub->publish(out);
      // A fresh all-unknown grid is a valid observation. The costmap decides
      // whether unknown cells retain surveyed terrain or stay closed.
      ok.data = true;
      last_surface_stamp = rclcpp::Time(msg->header.stamp);
      last_valid_surface = ok.data;
      publishReady();
    } catch (const tf2::TransformException &e) {
      publishReady();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Terrain TF: %s",
                           e.what());
    }
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Terrain>());
  rclcpp::shutdown();
}
