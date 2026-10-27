#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>

class ColorFilter : public rclcpp::Node {
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Policy = message_filters::sync_policies::ExactTime<Cloud, Cloud>;
  message_filters::Subscriber<Cloud> original_, filtered_;
  message_filters::Synchronizer<Policy> sync_;
  rclcpp::Publisher<Cloud>::SharedPtr pub_;
  bool layout(const Cloud &cloud) const {
    if (cloud.height != 1 || cloud.is_bigendian || !cloud.point_step ||
        cloud.row_step < uint64_t(cloud.width) * cloud.point_step ||
        cloud.data.size() < uint64_t(cloud.width) * cloud.point_step)
      return false;
    for (const char *name : {"x", "y", "z"}) {
      auto it = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                             [name](const auto &f) { return f.name == name; });
      if (it == cloud.fields.end() ||
          it->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
          it->count != 1 || uint64_t(it->offset) + 4 > cloud.point_step)
        return false;
    }
    return true;
  }

public:
  ColorFilter()
      : Node("odin_color_filter"), sync_(Policy(5), original_, filtered_) {
    auto qos = rclcpp::QoS(5).reliable().get_rmw_qos_profile();
    original_.subscribe(this, "/odin1/cloud_slam", qos);
    filtered_.subscribe(this, "/odin_navigation/cloud", qos);
    sync_.registerCallback(std::bind(&ColorFilter::process, this,
                                     std::placeholders::_1,
                                     std::placeholders::_2));
    pub_ = create_publisher<Cloud>("/odin_3d/color_filtered", 5);
  }
  void process(const Cloud::ConstSharedPtr &original,
               const Cloud::ConstSharedPtr &filtered) {
    const double age = (now() - rclcpp::Time(original->header.stamp)).seconds();
    if (age < -.05 || age > .3 || original->header.frame_id != "odom" ||
        filtered->header.frame_id != "odin_local" || !layout(*original) ||
        !layout(*filtered) || filtered->width > original->width)
      return;
    Cloud out = *original;
    out.header = filtered->header;
    sensor_msgs::PointCloud2ConstIterator<float> x(*original, "x"),
        y(*original, "y"), z(*original, "z");
    sensor_msgs::PointCloud2ConstIterator<float> fx(*filtered, "x"),
        fy(*filtered, "y"), fz(*filtered, "z");
    size_t i = 0;
    // The adapter preserves XYZ values and point order. Reuse its selection,
    // copying complete original records to keep RGB and any additional fields.
    for (size_t j = 0; j < filtered->width; j++, ++fx, ++fy, ++fz) {
      while (i < original->width && (*x != *fx || *y != *fy || *z != *fz)) {
        i++;
        ++x;
        ++y;
        ++z;
      }
      if (i == original->width) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Filtered cloud is not an ordered subset; skipping RGB frame");
        return;
      }
      std::memcpy(out.data.data() + j * out.point_step,
                  original->data.data() + i * original->point_step,
                  out.point_step);
      i++;
      ++x;
      ++y;
      ++z;
    }
    sensor_msgs::PointCloud2Modifier(out).resize(filtered->width);
    out.is_dense = true;
    if ((now() - rclcpp::Time(original->header.stamp)).seconds() <= .3)
      pub_->publish(out);
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorFilter>());
  rclcpp::shutdown();
}
