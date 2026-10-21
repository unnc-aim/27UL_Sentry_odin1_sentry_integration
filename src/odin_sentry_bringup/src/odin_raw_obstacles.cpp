#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "odin_cloud_filter.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdinRawObstacles : public rclcpp::Node
{
  using Odom = nav_msgs::msg::Odometry;
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Policy = message_filters::sync_policies::ExactTime<Odom, Odom>;
  struct Pose
  {
    int64_t time;
    tf2::Transform value;
  };
  struct Point
  {
    tf2::Vector3 value;
    float offset;
  };
  struct Frame
  {
    int64_t start, end;
    std::vector<Point> points;
    std::map<float, tf2::Transform> groups;
  };

public:
  OdinRawObstacles() : Node("odin_raw_obstacles")
  {
    max_age_ = positive("max_stamp_age", 0.3);
    max_future_ = positive("max_future_stamp", 0.05);
    max_gap_ = positive("max_pose_gap", 0.02);
    max_range_ = positive("max_range", 3.0);
    position_tolerance_ = positive("pose_position_tolerance", 0.05);
    angle_tolerance_ = positive("pose_angle_tolerance", 0.05);
    confidence_ = declare_parameter<int>("min_confidence", 35);
    if (confidence_ < 1 || confidence_ > 65535) {
      throw std::invalid_argument("min_confidence requires 1 to 65535");
    }
    filter_ = std::make_unique<OdinCloudFilter>(*this);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    cloud_pub_ = create_publisher<Cloud>("/odin_obstacles/registered_scan", 5);
    odom_pub_ = create_publisher<Odom>("/odin_obstacles/lidar_odometry", 5);
    poses_sub_ = create_subscription<Odom>(
      "/odin1/odometry_highfreq", rclcpp::QoS(2000),
      [this](Odom::ConstSharedPtr msg) { receivePose(*msg); });
    extrinsic_sub_ = create_subscription<Odom>(
      "/odin1/wiwc", 5, [this](Odom::ConstSharedPtr msg) { receiveExtrinsic(*msg); });
    cloud_sub_ = create_subscription<Cloud>(
      "/odin1/cloud_raw", 5, [this](Cloud::ConstSharedPtr msg) { receiveCloud(*msg); });
    const auto qos = rclcpp::QoS(20).get_rmw_qos_profile();
    device_sub_.subscribe(this, "/odin_navigation/odometry", qos);
    aligned_sub_.subscribe(this, "/lidar_odometry", qos);
    sync_ = std::make_unique<message_filters::Synchronizer<Policy>>(
      Policy(20), device_sub_, aligned_sub_);
    sync_->registerCallback(std::bind(
      &OdinRawObstacles::receiveAlignment, this, std::placeholders::_1, std::placeholders::_2));
    processing_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(5), [this]() { process(); }, processing_group_);
  }

private:
  double positive(const char * name, double value)
  {
    value = declare_parameter(name, value);
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument(std::string(name) + " requires a finite positive value");
    }
    return value;
  }

  bool fresh(int64_t time) const
  {
    const double age = (now().nanoseconds() - time) * 1e-9;
    return age <= max_age_ && age >= -max_future_;
  }

  void fault(const char * reason)
  {
    if (!faulted_) {
      RCLCPP_ERROR(
        get_logger(), "%s; raw output paused. Restart sensing and localization together.", reason);
    }
    faulted_ = true;
    frames_.clear();
  }

  static bool pose(const Odom & msg, tf2::Transform & out)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(norm) ||
      std::abs(norm - 1.0) > 0.01) {
      return false;
    }
    tf2::fromMsg(msg.pose.pose, out);
    auto rotation = out.getRotation();
    out.setRotation(rotation.normalized());
    return true;
  }

  static bool close(const tf2::Transform & a, const tf2::Transform & b, double p, double q)
  {
    return (a.getOrigin() - b.getOrigin()).length() <= p &&
           a.getRotation().angleShortestPath(b.getRotation()) <= q;
  }

  void receivePose(const Odom & msg)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (faulted_) {
      return;
    }
    const int64_t time = rclcpp::Time(msg.header.stamp).nanoseconds();
    tf2::Transform value;
    if (
      msg.header.frame_id != "odom" || msg.child_frame_id != "imu" || !fresh(time) ||
      !pose(msg, value)) {
      return;
    }
    if (!poses_.empty()) {
      const double dt = (time - poses_.back().time) * 1e-9;
      // PTP updates can move a high-frequency sample back by a fraction of a millisecond.
      // Drop that sample; interpolate only between strictly ordered samples.
      if (dt < -max_gap_) {
        fault("High-frequency time moved backwards");
        return;
      }
      if (dt <= 0.0) {
        return;
      }
      if (!close(value, poses_.back().value, 1.0, 1.5)) {
        fault("High-frequency pose changed discontinuously");
        return;
      }
    }
    poses_.push_back({time, value});
    while (poses_.size() > 2000 || (time - poses_.front().time) > 2000000000LL) {
      poses_.pop_front();
    }
  }

  bool interpolate(int64_t time, tf2::Transform & out) const
  {
    auto right = std::lower_bound(
      poses_.begin(), poses_.end(), time, [](const Pose & p, int64_t t) { return p.time < t; });
    if (right == poses_.end()) {
      return false;
    }
    if (right->time == time) {
      out = right->value;
      return true;
    }
    if (right == poses_.begin()) {
      return false;
    }
    const auto left = std::prev(right);
    const auto span = right->time - left->time;
    if (span * 1e-9 > max_gap_) {
      return false;
    }
    const double u = static_cast<double>(time - left->time) / span;
    out.setOrigin(left->value.getOrigin().lerp(right->value.getOrigin(), u));
    out.setRotation(left->value.getRotation().slerp(right->value.getRotation(), u));
    return true;
  }

  void receiveAlignment(
    const Odom::ConstSharedPtr & device_msg, const Odom::ConstSharedPtr & aligned_msg)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    const auto & device = *device_msg;
    const auto & aligned = *aligned_msg;
    tf2::Transform d, a;
    const int64_t time = rclcpp::Time(device.header.stamp).nanoseconds();
    if (
      faulted_ || !fresh(time) || device.header.frame_id != "odin_local" ||
      device.child_frame_id != "odin_imu" || aligned.header.frame_id != "odom" ||
      aligned.child_frame_id != "odin_imu" || !pose(device, d) || !pose(aligned, a)) {
      return;
    }
    const auto transform = a * d.inverse();
    if (have_alignment_ && !close(alignment_, transform, 0.001, 0.001)) {
      fault("Navigation alignment changed");
      return;
    }
    alignment_ = transform;
    reference_ = {time, d};
    have_alignment_ = true;
  }

  void receiveExtrinsic(const Odom & msg)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (faulted_) {
      return;
    }
    const auto & m = msg.twist.covariance;
    for (size_t i = 0; i < 16; ++i) {
      if (!std::isfinite(m[i])) {
        return;
      }
    }
    if (std::abs(m[12]) + std::abs(m[13]) + std::abs(m[14]) + std::abs(m[15] - 1) > 1e-6) {
      return;
    }
    tf2::Matrix3x3 rotation(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
    const auto product = rotation.transpose() * rotation;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        if (std::abs(product[i][j] - (i == j ? 1.0 : 0.0)) > 1e-5) {
          return;
        }
      }
    }
    if (std::abs(rotation.determinant() - 1.0) > 1e-5) {
      return;
    }
    const tf2::Transform extrinsic(rotation, tf2::Vector3(m[3], m[7], m[11]));
    if (extrinsic.getOrigin().length() > 1.0) {
      return;
    }
    const int64_t time = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (!fresh(time)) {
      return;
    }
    if (!extrinsics_.empty()) {
      if (time <= extrinsics_.back().time) {
        return;
      }
      if (!close(extrinsics_.back().value, extrinsic, 0.05, 0.1)) {
        fault("Device extrinsic changed discontinuously");
        return;
      }
    } else {
      RCLCPP_INFO(get_logger(), "Receiving online device T_IL calibration");
    }
    extrinsics_.push_back({time, extrinsic});
    while (extrinsics_.size() > 100 || time - extrinsics_.front().time > 2000000000LL) {
      extrinsics_.pop_front();
    }
  }

  bool extrinsicAt(int64_t time, tf2::Transform & out) const
  {
    // T_IL is updated online by the device. Use the latest calibration available
    // at each exposure; these updates describe calibration, not sensor motion.
    auto next = std::upper_bound(
      extrinsics_.begin(), extrinsics_.end(), time,
      [](int64_t t, const Pose & p) { return t < p.time; });
    if (next == extrinsics_.begin()) {
      return false;
    }
    const auto sample = std::prev(next);
    if ((time - sample->time) * 1e-9 > max_age_) {
      return false;
    }
    out = sample->value;
    return true;
  }

  template <typename T>
  static T read(const uint8_t * data, size_t offset)
  {
    T value;
    std::memcpy(&value, data + offset, sizeof(T));
    return value;
  }

  void receiveCloud(const Cloud & msg)
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    const int64_t start = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (
      faulted_ || !fresh(start) || msg.header.frame_id != "lidar" || msg.is_bigendian ||
      msg.height != 1 || msg.width == 0 || msg.width > 1000000 ||
      msg.row_step < uint64_t(msg.width) * msg.point_step || msg.data.size() < msg.row_step) {
      return;
    }
    std::vector<size_t> offsets;
    for (const auto & name : {"x", "y", "z", "confidence", "offset_time"}) {
      const auto field = std::find_if(
        msg.fields.begin(), msg.fields.end(), [name](const auto & f) { return f.name == name; });
      const bool confidence = std::string(name) == "confidence";
      const auto type =
        confidence ? sensor_msgs::msg::PointField::UINT16 : sensor_msgs::msg::PointField::FLOAT32;
      const auto size = confidence ? sizeof(uint16_t) : sizeof(float);
      if (
        field == msg.fields.end() || field->datatype != type || field->count != 1 ||
        uint64_t(field->offset) + size > msg.point_step) {
        return;
      }
      offsets.push_back(field->offset);
    }
    Frame frame{start, start, {}, {}};
    frame.points.reserve(msg.width);
    size_t valid_returns = 0;
    for (size_t i = 0; i < msg.width; ++i) {
      const auto data = msg.data.data() + i * msg.point_step;
      const float x = read<float>(data, offsets[0]), y = read<float>(data, offsets[1]),
                  z = read<float>(data, offsets[2]);
      if (
        !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        (x == 0 && y == 0 && z == 0)) {
        continue;
      }
      if (read<uint16_t>(data, offsets[3]) < confidence_) {
        continue;
      }
      const float offset = read<float>(data, offsets[4]);
      if (!std::isfinite(offset) || offset < 0 || offset > 0.15) {
        return;
      }
      ++valid_returns;
      frame.end = std::max(frame.end, start + static_cast<int64_t>(std::llround(offset * 1e9)));
      frame.groups.emplace(offset, tf2::Transform::getIdentity());
      if (frame.groups.size() > 64) {
        return;
      }
      const tf2::Vector3 point(x, y, z);
      if (point.length2() <= max_range_ * max_range_) {
        frame.points.push_back({point, offset});
      }
    }
    // A scan without usable returns keeps the obstacle observation out of date.
    if (valid_returns == 0 || !fresh(frame.end)) {
      return;
    }
    if (last_start_ != 0 && start <= last_start_) {
      if (start < last_start_ - 20000000LL) {
        fault("Raw cloud time moved backwards");
      }
      return;
    }
    last_start_ = start;
    frames_.push_back(std::move(frame));
    while (frames_.size() > 1) {
      frames_.pop_front();
    }
  }

  void process()
  {
    std::unique_lock<std::mutex> lock(input_mutex_);
    if (faulted_) {
      return;
    }
    while (!frames_.empty() && !fresh(frames_.front().start)) {
      frames_.pop_front();
    }
    if (frames_.empty()) {
      return;
    }
    tf2::Transform check;
    if (
      !have_alignment_ || extrinsics_.empty() || !fresh(reference_.time) ||
      !interpolate(reference_.time, check) ||
      !close(check, reference_.value, position_tolerance_, angle_tolerance_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for calibration, aligned SLAM pose and consistent high-frequency pose");
      return;
    }
    auto & pending = frames_.front();
    tf2::Transform end_pose;
    if (!interpolate(pending.end, end_pose)) {
      return;
    }
    for (auto & group : pending.groups) {
      const auto time = pending.start + static_cast<int64_t>(std::llround(group.first * 1e9));
      tf2::Transform device_pose;
      if (!interpolate(time, device_pose)) {
        return;
      }
      tf2::Transform extrinsic;
      if (!extrinsicAt(time, extrinsic)) {
        return;
      }
      group.second = alignment_ * device_pose * extrinsic;
    }
    auto frame = std::move(pending);
    frames_.pop_front();
    Cloud output;
    output.header.stamp = rclcpp::Time(frame.end);
    output.header.frame_id = "odom";
    output.height = 1;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(frame.points.size());
    sensor_msgs::PointCloud2Iterator<float> x(output, "x"), y(output, "y"), z(output, "z"),
      intensity(output, "intensity");
    size_t count = 0;
    for (const auto & point : frame.points) {
      tf2::Transform extrinsic;
      extrinsicAt(frame.start + static_cast<int64_t>(std::llround(point.offset * 1e9)), extrinsic);
      const auto local = extrinsic * point.value;
      if (!filter_->keep(local)) {
        continue;
      }
      const auto world = frame.groups.at(point.offset) * point.value;
      *x = world.x();
      *y = world.y();
      *z = world.z();
      *intensity = 0.0;
      ++x;
      ++y;
      ++z;
      ++intensity;
      ++count;
    }
    modifier.resize(count);
    output.is_dense = true;
    const auto aligned = alignment_ * end_pose;
    tf2::Transform end_extrinsic;
    if (!extrinsicAt(frame.end, end_extrinsic)) {
      return;
    }
    // Density filtering can take most of a sensor period. Keep pose and SLAM
    // reference callbacks running so their queues and timestamps stay current.
    lock.unlock();
    filter_->apply(output, aligned.getOrigin());
    lock.lock();
    if (!faulted_ && fresh(frame.start) && fresh(reference_.time)) {
      Odom odom;
      odom.header = output.header;
      odom.child_frame_id = "odin_imu";
      tf2::toMsg(aligned, odom.pose.pose);
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = output.header.stamp;
      tf.header.frame_id = "odin_imu";
      tf.child_frame_id = "odin_lidar";
      tf.transform = tf2::toMsg(end_extrinsic);
      broadcaster_->sendTransform(tf);
      odom_pub_->publish(odom);
      cloud_pub_->publish(output);
      if (!published_) {
        RCLCPP_INFO(
          get_logger(), "Raw obstacle input ready: %zu groups, %.3f s, %u points",
          frame.groups.size(), (frame.end - frame.start) * 1e-9, output.width);
        published_ = true;
      }
    }
  }

  std::mutex input_mutex_;
  rclcpp::CallbackGroup::SharedPtr processing_group_;
  double max_age_, max_future_, max_gap_, max_range_, position_tolerance_, angle_tolerance_;
  int confidence_;
  bool faulted_{false}, have_alignment_{false}, published_{false};
  int64_t last_start_{0};
  Pose reference_{};
  tf2::Transform alignment_;
  std::deque<Pose> poses_, extrinsics_;
  std::deque<Frame> frames_;
  std::unique_ptr<OdinCloudFilter> filter_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_;
  rclcpp::Subscription<Odom>::SharedPtr poses_sub_, extrinsic_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  message_filters::Subscriber<Odom> device_sub_, aligned_sub_;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdinRawObstacles>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
