#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "odin_cloud_filter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class OdinNavAdapter : public rclcpp::Node
{
public:
  OdinNavAdapter() : Node("odin_nav_adapter")
  {
    max_age_ = declare_parameter("max_stamp_age", 0.3);
    max_future_ = declare_parameter("max_future_stamp", 0.05);
    max_pair_dt_ = declare_parameter("max_pair_dt", 0.05);
    timeout_ = declare_parameter("input_timeout", 0.5);
    recovery_timeout_ = declare_parameter("recovery_timeout", 2.0);
    max_translation_step_ = declare_parameter("max_translation_step", 1.0);
    max_rotation_step_ = declare_parameter("max_rotation_step", 1.5);
    for (double value :
         {max_age_, max_future_, max_pair_dt_, timeout_, recovery_timeout_, max_translation_step_,
          max_rotation_step_}) {
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument("Input limits must be finite and positive");
      }
    }
    if (recovery_timeout_ <= timeout_) {
      throw std::invalid_argument("Recovery timeout must exceed input timeout");
    }
    declare_parameter("base_frame", "base_footprint");
    lidar_frame_ = declare_parameter("lidar_frame", "odin_imu");
    independent_odometry_ = declare_parameter("independent_odometry", false);
    cloud_filter_ = std::make_unique<OdinCloudFilter>(*this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>("odin_navigation/odometry", 5);
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("odin_navigation/cloud", 5);
    // Prefer fresh odometry; retain retransmission for fragmented clouds.
    auto qos = rclcpp::QoS(5).reliable().get_rmw_qos_profile();
    odometry_sub_.subscribe(this, "odin1/odometry", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    if (independent_odometry_) {
      odometry_sub_.registerCallback(
        std::bind(&OdinNavAdapter::receiveOdometry, this, std::placeholders::_1));
    }
    cloud_sub_.subscribe(this, "odin1/cloud_slam", qos);
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(20), odometry_sub_, cloud_sub_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_pair_dt_));
    // These Odin streams run at 10 Hz. A 10 ms lower bound lets pairs with
    // submillisecond PTP corrections complete before the next sensor frame.
    sync_->setInterMessageLowerBound(0, rclcpp::Duration::from_seconds(0.01));
    sync_->setInterMessageLowerBound(1, rclcpp::Duration::from_seconds(0.01));
    if (independent_odometry_) {
      sync_->registerCallback(
        std::bind(&OdinNavAdapter::queueCloud, this, std::placeholders::_1, std::placeholders::_2));
      cloud_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      cloud_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {
        nav_msgs::msg::Odometry::ConstSharedPtr odometry;
        sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          odometry = std::move(pending_odometry_);
          cloud = std::move(pending_cloud_);
        }
        if (odometry && cloud) {
          process(odometry, cloud);
        }
      }, cloud_group_);
    } else {
      sync_->registerCallback(
        std::bind(&OdinNavAdapter::process, this, std::placeholders::_1, std::placeholders::_2));
    }
    aligned_odometry_sub_.subscribe(this, "lidar_odometry", qos);
    anchor_sync_ = std::make_unique<message_filters::Synchronizer<AnchorPolicy>>(
      AnchorPolicy(20), odometry_sub_, aligned_odometry_sub_);
    anchor_sync_->registerCallback(
      std::bind(&OdinNavAdapter::publishAnchor, this,
                std::placeholders::_1, std::placeholders::_2));
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (independent_odometry_) {
        if (odometry_initialized_ && !faulted_) {
          const double elapsed =
            std::chrono::duration<double>(Clock::now() - odometry_receive_).count();
          if (elapsed > timeout_) {
            odometry_paused_ = true;
            odometry_recovery_samples_ = 0;
          }
        }
        return;
      }
      if (initialized_ && !faulted_) {
        const double elapsed = std::chrono::duration<double>(Clock::now() - last_receive_).count();
        if (elapsed > recovery_timeout_) {
          stop("Synchronized input recovery expired");
        } else if (elapsed > timeout_) {
          pause();
        }
      }
    });
  }

private:
  using Clock = std::chrono::steady_clock;
  using AnchorPolicy = message_filters::sync_policies::ExactTime<
    nav_msgs::msg::Odometry, nav_msgs::msg::Odometry>;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

  void queueCloud(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_odometry_ = odometry;
    pending_cloud_ = cloud;
  }

  static bool validPose(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return msg.header.frame_id == "odom" && msg.child_frame_id == "imu" &&
           std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(norm) && std::abs(norm - 1.0) <= 0.01;
  }

  void publishOdometry(const nav_msgs::msg::Odometry & msg, const tf2::Transform & pose)
  {
    nav_msgs::msg::Odometry out;
    out.header = msg.header;
    out.header.frame_id = "odin_local";
    out.child_frame_id = "odin_imu";
    out.pose.pose = msg.pose.pose;
    out.pose.pose.orientation = tf2::toMsg(pose.getRotation());
    odometry_pub_->publish(out);
  }

  void receiveOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (faulted_) {
      return;
    }
    const double age = (now() - rclcpp::Time(msg->header.stamp)).seconds();
    if (age < -max_future_ || age > recovery_timeout_) {
      odometry_recovery_samples_ = 0;
      return;
    }
    if (age > max_age_ && odometry_initialized_ &&
        rclcpp::Time(msg->header.stamp) <= odometry_stamp_) {
      odometry_recovery_samples_ = 0;
      return;
    }
    if (!validPose(*msg)) {
      if (odometry_initialized_) {
        stop("Odometry frame or pose changed");
      }
      return;
    }
    tf2::Transform pose;
    tf2::fromMsg(msg->pose.pose, pose);
    pose.setRotation(pose.getRotation().normalized());
    const rclcpp::Time stamp(msg->header.stamp);
    if (odometry_initialized_) {
      const double dt = (stamp - odometry_stamp_).seconds();
      const double elapsed =
        std::chrono::duration<double>(Clock::now() - odometry_receive_).count();
      const double translation = (pose.getOrigin() - odometry_pose_.getOrigin()).length();
      const double rotation = pose.getRotation().angleShortestPath(odometry_pose_.getRotation());
      if (dt < 0.0 || translation > max_translation_step_ || rotation > max_rotation_step_) {
        RCLCPP_ERROR(get_logger(),
          "Odometry discontinuity: dt=%.6fs receive_gap=%.3fs translation=%.3fm rotation=%.3frad",
          dt, elapsed, translation, rotation);
        stop("Odometry continuity changed");
        return;
      }
      if (dt == 0.0) {
        return;
      }
      if (dt > timeout_ || elapsed > timeout_) {
        odometry_paused_ = true;
        odometry_recovery_samples_ = 0;
      }
    }
    odometry_initialized_ = true;
    odometry_stamp_ = stamp;
    odometry_pose_ = pose;
    // Keep the observed trajectory for continuity checks across delayed delivery.
    // Delayed samples never refresh published pose time or the recovery count.
    if (!validStamp(msg->header.stamp)) {
      odometry_recovery_samples_ = 0;
      return;
    }
    odometry_receive_ = Clock::now();
    if (odometry_paused_) {
      if (++odometry_recovery_samples_ < 3) {
        return;
      }
      odometry_paused_ = false;
    }
    // Native navigation separately checks fresh raw terrain before motion.
    // Preserve measurement time and publish before waiting for the RGB SLAM cloud.
    publishOdometry(*msg, pose);
  }

  void pause()
  {
    if (!paused_) {
      RCLCPP_WARN(get_logger(), "Synchronized input paused; waiting for three fresh continuous pairs");
    }
    paused_ = true;
    recovery_samples_ = 0;
  }

  void stop(const char * reason)
  {
    if (!faulted_) {
      RCLCPP_ERROR(
        get_logger(), "%s; output paused. Restart sensing and localization together.", reason);
    }
    faulted_ = true;
  }

  bool validStamp(const builtin_interfaces::msg::Time & stamp) const
  {
    const double age = (now() - rclcpp::Time(stamp)).seconds();
    return age >= -max_future_ && age <= max_age_;
  }

  bool validCloud(const sensor_msgs::msg::PointCloud2 & cloud) const
  {
    const uint64_t row_size = static_cast<uint64_t>(cloud.width) * cloud.point_step;
    if (
      cloud.height != 1 || cloud.width == 0 || cloud.is_bigendian || cloud.row_step < row_size ||
      cloud.data.size() < row_size) {
      return false;
    }
    for (const char * name : {"x", "y", "z"}) {
      const auto found = std::find_if(
        cloud.fields.begin(), cloud.fields.end(),
        [name](const auto & field) { return field.name == name; });
      if (
        found == cloud.fields.end() || found->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        found->count != 1 ||
        static_cast<uint64_t>(found->offset) + sizeof(float) > cloud.point_step) {
        return false;
      }
    }
    return true;
  }

  void process(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud)
  {
    std::unique_lock<std::mutex> lock(state_mutex_);
    if (faulted_) {
      return;
    }
    if (!validStamp(odometry->header.stamp) || !validStamp(cloud->header.stamp)) {
      recovery_samples_ = 0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for fresh Odin timestamps (odom %.3fs, cloud %.3fs)",
        (now() - rclcpp::Time(odometry->header.stamp)).seconds(),
        (now() - rclcpp::Time(cloud->header.stamp)).seconds());
      return;
    }
    if (cloud->width == 0) {
      recovery_samples_ = 0;
      return;
    }
    if (
      !validPose(*odometry) || cloud->header.frame_id != "odom" ||
      !validCloud(*cloud)) {
      if (initialized_) {
        stop("Input frame, pose or point layout changed");
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000, "Waiting for valid Odin pose and XYZ cloud");
      }
      return;
    }

    tf2::Transform pose;
    tf2::fromMsg(odometry->pose.pose, pose);
    auto rotation = pose.getRotation();
    rotation.normalize();
    pose.setRotation(rotation);
    const rclcpp::Time stamp(odometry->header.stamp);
    const rclcpp::Time cloud_stamp(cloud->header.stamp);
    if (initialized_) {
      const double dt = (stamp - last_stamp_).seconds();
      const double cloud_dt = (cloud_stamp - last_cloud_stamp_).seconds();
      if (
        dt < 0.0 || cloud_dt < 0.0 ||
        (!independent_odometry_ &&
         (dt > recovery_timeout_ || cloud_dt > recovery_timeout_ ||
          (pose.getOrigin() - previous_pose_.getOrigin()).length() > max_translation_step_ ||
          pose.getRotation().angleShortestPath(previous_pose_.getRotation()) > max_rotation_step_))) {
        stop("Odometry continuity changed");
        return;
      }
      if (dt == 0.0 || cloud_dt == 0.0) {
        recovery_samples_ = 0;
        return;
      }
      if (dt > timeout_ || cloud_dt > timeout_ ||
          std::chrono::duration<double>(Clock::now() - last_receive_).count() > timeout_) {
        pause();
      }
    }

    const tf2::Transform world_to_imu = pose.inverse();
    // Filtering has its own callback group; pose and alignment callbacks continue.
    lock.unlock();

    sensor_msgs::msg::PointCloud2 out_cloud;
    out_cloud.header = cloud->header;
    out_cloud.header.frame_id = "odin_local";
    out_cloud.height = 1;
    sensor_msgs::PointCloud2Modifier modifier(out_cloud);
    modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(cloud->width);
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y"), z(*cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> ox(out_cloud, "x"), oy(out_cloud, "y"),
      oz(out_cloud, "z"), intensity(out_cloud, "intensity");
    size_t count = 0;
    for (size_t i = 0; i < cloud->width; ++i, ++x, ++y, ++z) {
      if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
        continue;
      }
      if (!cloud_filter_->keep(world_to_imu * tf2::Vector3(*x, *y, *z))) {
        continue;
      }
      *ox = *x;
      *oy = *y;
      *oz = *z;
      *intensity = 0.0f;
      ++ox;
      ++oy;
      ++oz;
      ++intensity;
      ++count;
    }
    modifier.resize(count);
    out_cloud.is_dense = true;
    cloud_filter_->apply(out_cloud, pose.getOrigin());
    lock.lock();
    if (faulted_ || !validStamp(odometry->header.stamp) || !validStamp(cloud->header.stamp)) {
      recovery_samples_ = 0;
      return;
    }
    // An all-body frame still carries a valid pose and an empty filtered cloud.

    if (!initialized_) {
      initialized_ = true;
      RCLCPP_INFO(get_logger(), "Synchronized Odin navigation input is ready");
    }
    previous_pose_ = pose;
    last_stamp_ = stamp;
    last_cloud_stamp_ = cloud_stamp;
    last_receive_ = Clock::now();
    if (paused_) {
      if (++recovery_samples_ < 3) {
        return;
      }
      paused_ = false;
      RCLCPP_INFO(get_logger(), "Synchronized input recovered after three fresh continuous pairs");
    }
    if (!independent_odometry_) {
      publishOdometry(*odometry, pose);
    }
    lock.unlock();
    cloud_pub_->publish(out_cloud);
  }

  // Share the alignment actually selected by loam_interface. Selecting an
  // independent startup pose/mount sample can rotate RGB relative to raw clouds.
  void publishAnchor(
    const nav_msgs::msg::Odometry::ConstSharedPtr & device,
    const nav_msgs::msg::Odometry::ConstSharedPtr & aligned)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (faulted_ || anchor_published_ ||
        device->header.frame_id != "odom" || device->child_frame_id != "imu" ||
        aligned->header.frame_id != "odom" || aligned->child_frame_id != lidar_frame_ ||
        !validStamp(device->header.stamp)) {
      return;
    }
    for (const auto * msg : {device.get(), aligned.get()}) {
      const auto & p = msg->pose.pose.position;
      const auto & q = msg->pose.pose.orientation;
      const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01) {
        return;
      }
    }
    tf2::Transform device_pose, aligned_pose;
    tf2::fromMsg(device->pose.pose, device_pose);
    tf2::fromMsg(aligned->pose.pose, aligned_pose);
    device_pose.setRotation(device_pose.getRotation().normalized());
    aligned_pose.setRotation(aligned_pose.getRotation().normalized());
    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = device->header.stamp;
    out.header.frame_id = "odom";
    out.child_frame_id = "odin_local";
    out.transform = tf2::toMsg(aligned_pose * device_pose.inverse());
    static_broadcaster_->sendTransform(out);
    anchor_published_ = true;
    RCLCPP_INFO(get_logger(), "Anchored odin_local using the loam_interface alignment");
  }

  double max_age_, max_future_, max_pair_dt_, timeout_, recovery_timeout_;
  std::mutex state_mutex_;
  bool independent_odometry_{false}, odometry_initialized_{false}, odometry_paused_{false};
  unsigned odometry_recovery_samples_{0};
  rclcpp::Time odometry_stamp_{0, 0, RCL_ROS_TIME};
  Clock::time_point odometry_receive_;
  tf2::Transform odometry_pose_;
  nav_msgs::msg::Odometry::ConstSharedPtr pending_odometry_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  rclcpp::CallbackGroup::SharedPtr cloud_group_;
  rclcpp::TimerBase::SharedPtr cloud_timer_;
  double max_translation_step_, max_rotation_step_;
  std::string lidar_frame_;
  std::unique_ptr<OdinCloudFilter> cloud_filter_;
  bool anchor_published_{false};
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  bool initialized_{false};
  bool faulted_{false};
  bool paused_{false};
  unsigned recovery_samples_{0};
  tf2::Transform previous_pose_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME}, last_cloud_stamp_{0, 0, RCL_ROS_TIME};
  Clock::time_point last_receive_;
  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_, aligned_odometry_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  std::unique_ptr<message_filters::Synchronizer<AnchorPolicy>> anchor_sync_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdinNavAdapter>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
