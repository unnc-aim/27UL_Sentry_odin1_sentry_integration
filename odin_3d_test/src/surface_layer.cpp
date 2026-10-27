#include <algorithm>
#include <cmath>
#include <mutex>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace odin_3d_test {
class SurfaceLayer : public nav2_costmap_2d::Layer {
  std::mutex mutex;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
  double max_age = 0.5, rx = 0, ry = 0, ryaw = 0;
  bool close_unknown = true;
  double footprint_padding = 0.0;
  int64_t last_stamp = 0;

public:
  void onInitialize() override {
    auto node = node_.lock();
    if (!node)
      throw std::runtime_error("Costmap node expired");
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + ".enabled", enabled_);
    declareParameter("close_unknown", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + ".close_unknown", close_unknown);
    node->get_parameter("footprint_padding", footprint_padding);
    sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/odin_3d/surface", rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) {
          if (m->header.frame_id != "odom" || m->info.resolution <= 0 ||
              !std::isfinite(m->info.resolution) ||
              m->data.size() != size_t(m->info.width) * m->info.height)
            return;
          std::lock_guard<std::mutex> lock(mutex);
          auto node = node_.lock();
          const auto stamp = rclcpp::Time(m->header.stamp);
          if (!node || stamp.nanoseconds() <= last_stamp)
            return;
          const double age = (node->now() - stamp).seconds();
          if (age < -.05 || age > max_age)
            return;
          last_stamp = stamp.nanoseconds();
          grid = m;
        });
    current_ = false;
  }
  bool isClearable() override { return false; }
  void reset() override {
    std::lock_guard<std::mutex> lock(mutex);
    grid.reset();
    current_ = false;
  }
  void updateBounds(double x, double y, double yaw, double *x0, double *y0,
                    double *x1, double *y1) override {
    rx = x;
    ry = y;
    ryaw = yaw;
    auto c = layered_costmap_->getCostmap();
    *x0 = std::min(*x0, c->getOriginX());
    *y0 = std::min(*y0, c->getOriginY());
    *x1 = std::max(*x1, c->getOriginX() + c->getSizeInMetersX());
    *y1 = std::max(*y1, c->getOriginY() + c->getSizeInMetersY());
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &master, int x0, int y0, int x1,
                   int y1) override {
    if (!enabled_)
      return;
    std::lock_guard<std::mutex> lock(mutex);
    auto node = node_.lock();
    if (!node)
      return;
    double age =
        grid ? (node->now() - rclcpp::Time(grid->header.stamp)).seconds() : 1e9;
    current_ = grid && age >= -.05 && age <= max_age;
    tf2::Transform transform;
    transform.setIdentity();
    if (layered_costmap_->getGlobalFrameID() != "odom") {
      try {
        tf2::fromMsg(tf_->lookupTransform("odom",
                                          layered_costmap_->getGlobalFrameID(),
                                          tf2::TimePointZero)
                         .transform,
                     transform);
      } catch (const tf2::TransformException &) {
        current_ = false;
      }
    }
    // Self filtering uses the physical footprint, excluding planning padding.
    // Nearby measured obstacles still participate in global route selection.
    auto physical_footprint = layered_costmap_->getFootprint();
    nav2_costmap_2d::padFootprint(physical_footprint, -footprint_padding);
    std::vector<geometry_msgs::msg::Point> footprint;
    nav2_costmap_2d::transformFootprint(rx, ry, ryaw, physical_footprint, footprint);
    std::vector<nav2_costmap_2d::MapLocation> polygon, filled;
    for (const auto &point : footprint) {
      nav2_costmap_2d::MapLocation cell;
      if (!master.worldToMap(point.x, point.y, cell.x, cell.y)) {
        polygon.clear();
        break;
      }
      polygon.push_back(cell);
    }
    std::vector<unsigned> own_cells;
    if (polygon.size() >= 3) {
      master.convexFillCells(polygon, filled);
      for (const auto &cell : filled)
        own_cells.push_back(master.getIndex(cell.x, cell.y));
      std::sort(own_cells.begin(), own_cells.end());
    }
    for (int y = y0; y < y1; y++)
      for (int x = x0; x < x1; x++) {
        double wx, wy;
        master.mapToWorld(x, y, wx, wy);
        int cost = -1;
        const auto p = transform * tf2::Vector3(wx, wy, 0);
        const unsigned index = master.getIndex(x, y);
        if (!own_cells.empty() && index >= own_cells.front() &&
            index <= own_cells.back() &&
            std::binary_search(own_cells.begin(), own_cells.end(), index)) {
          continue;
        }
        if (current_) {
          int gx = std::floor((p.x() - grid->info.origin.position.x) /
                              grid->info.resolution);
          int gy = std::floor((p.y() - grid->info.origin.position.y) /
                              grid->info.resolution);
          if (gx >= 0 && gy >= 0 && gx < int(grid->info.width) &&
              gy < int(grid->info.height))
            cost = grid->data[gy * grid->info.width + gx];
        }
        // Leave unseen regions outside the local observation window to the
        // static global map.
        if (cost < 0 && !close_unknown)
          continue;
        if (cost < 0 && std::hypot(wx - rx, wy - ry) <
                            layered_costmap_->getInscribedRadius())
          continue;
        unsigned char v = cost < 0 || cost >= 100
                              ? nav2_costmap_2d::LETHAL_OBSTACLE
                              : static_cast<unsigned char>(cost * 2.5);
        auto old = master.getCost(x, y);
        if (old == nav2_costmap_2d::NO_INFORMATION || v > old)
          master.setCost(x, y, v);
      }
  }
};
} // namespace odin_3d_test
PLUGINLIB_EXPORT_CLASS(odin_3d_test::SurfaceLayer, nav2_costmap_2d::Layer)
