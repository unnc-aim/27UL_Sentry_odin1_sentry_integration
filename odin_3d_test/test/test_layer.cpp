#include <cmath>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_loader.hpp>
#include <stdexcept>
#include <tf2_ros/static_transform_broadcaster.h>
#include <thread>
#include <sensor_msgs/point_cloud2_iterator.hpp>
int main(int argc, char **argv) {
  const bool surface_only = argc > 1 && std::string(argv[1]) == "--surface-only";
  rclcpp::init(argc, argv);
  // Keep plugin code loaded through destruction of the node callback groups.
  pluginlib::ClassLoader<nav2_costmap_2d::Layer> plugins(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
  for (const auto &name : {"nav2_costmap_2d::StaticLayer",
                           "nav2_costmap_2d::InflationLayer",
                           "odin_3d_test::SurfaceLayer",
                           "pb_nav2_costmap_2d::IntensityVoxelLayer"})
    plugins.loadLibraryForClass(name);
  auto n = std::make_shared<rclcpp::Node>("surface_test");
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> planners(
      "nav2_core", "nav2_core::GlobalPlanner");
  auto planner = planners.createSharedInstance("nav2_theta_star_planner/ThetaStarPlanner");
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("global_frame", "odom"),
       rclcpp::Parameter("robot_base_frame", "test_base"),
       rclcpp::Parameter("rolling_window", true), rclcpp::Parameter("width", 4),
       rclcpp::Parameter("height", 4), rclcpp::Parameter("resolution", .05),
       rclcpp::Parameter("update_frequency", 20.),
       rclcpp::Parameter("robot_radius", .2),
       rclcpp::Parameter("footprint_padding", surface_only ? .10 : .01),
       rclcpp::Parameter("track_unknown_space", true),
       rclcpp::Parameter("plugins", surface_only
           ? std::vector<std::string>{"static", "surface", "inflation"}
           : std::vector<std::string>{"static", "intensity", "surface", "inflation"}),
       rclcpp::Parameter("intensity.plugin", "pb_nav2_costmap_2d::IntensityVoxelLayer"),
       rclcpp::Parameter("intensity.min_obstacle_height", -2.),
       rclcpp::Parameter("intensity.max_obstacle_height", 2.),
       rclcpp::Parameter("intensity.origin_z", 0.),
       rclcpp::Parameter("intensity.z_resolution", .05),
       rclcpp::Parameter("intensity.observation_sources", "points"),
       rclcpp::Parameter("intensity.points.topic", "/height_test"),
       rclcpp::Parameter("intensity.points.data_type", "PointCloud2"),
       rclcpp::Parameter("intensity.points.sensor_frame", "test_base"),
       rclcpp::Parameter("intensity.points.min_obstacle_height", -2.),
       rclcpp::Parameter("intensity.points.max_obstacle_height", 2.),
       rclcpp::Parameter("static.plugin", "nav2_costmap_2d::StaticLayer"),
       rclcpp::Parameter("static.map_subscribe_transient_local", true),
       rclcpp::Parameter("surface.plugin", "odin_3d_test::SurfaceLayer"),
       rclcpp::Parameter("surface.close_unknown", false),
       rclcpp::Parameter("inflation.plugin", "nav2_costmap_2d::InflationLayer"),
       rclcpp::Parameter("inflation.inflate_around_unknown", true),
       rclcpp::Parameter("GridBased.allow_unknown", false),
       rclcpp::Parameter("GridBased.w_traversal_cost", 10.),
       rclcpp::Parameter("inflation.inflation_radius", .4)});
  auto cm = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
  rclcpp::executors::MultiThreadedExecutor ex;
  ex.add_node(n);
  ex.add_node(cm->get_node_base_interface());
  std::thread thread([&] { ex.spin(); });
  auto pub =
      n->create_publisher<nav_msgs::msg::OccupancyGrid>("/odin_3d/surface", 1);
  auto map_pub = n->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local());
  auto cloud_pub = n->create_publisher<sensor_msgs::msg::PointCloud2>("/height_test", 1);
  tf2_ros::StaticTransformBroadcaster br(n);
  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "test_base";
  tf.transform.rotation.w = 1.;
  br.sendTransform(tf);
  int result = 0;
  try {
    nav_msgs::msg::OccupancyGrid g;
    g.header.frame_id = "odom";
    g.info.resolution = .05;
    g.info.width = g.info.height = 80;
    g.info.origin.position.x = g.info.origin.position.y = -2.;
    g.info.origin.orientation.w = 1.;
    g.data.resize(g.info.width * g.info.height, 0);
    auto paint = [](nav_msgs::msg::OccupancyGrid &grid, double xmin, double ymin,
                    double xmax, double ymax, int8_t value) {
      for (unsigned y = 0; y < grid.info.height; ++y)
        for (unsigned x = 0; x < grid.info.width; ++x) {
          const double wx = grid.info.origin.position.x + (x + .5) * grid.info.resolution;
          const double wy = grid.info.origin.position.y + (y + .5) * grid.info.resolution;
          if (wx >= xmin && wx < xmax && wy >= ymin && wy < ymax)
            grid.data[y * grid.info.width + x] = value;
        }
    };
    auto surveyed = g;
    paint(surveyed, 1., 1., 1.1, 1.1, 100);
    paint(surveyed, -1., 1., -.9, 1.1, -1);
    surveyed.header.stamp = n->now();
    map_pub->publish(surveyed);
    cm->configure();
    cm->activate();
    planner->configure(cm, "GridBased", cm->getTfBuffer(), cm);
    planner->activate();
    auto publish_surface = [&]() {
      for (int i = 0; i < 10; i++) {
        g.header.stamp = n->now();
        pub->publish(g);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    };
    auto send = [&](int8_t v) {
      std::fill(g.data.begin(), g.data.end(), v);
      for (int i = 0; i < 20; i++) {
        g.header.stamp = n->now();
        pub->publish(g);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    };
    auto cell = [&](double wx = 1., double wy = 0.) {
      auto map = cm->getCostmap();
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
          *map->getMutex());
      unsigned x, y;
      if (!map->worldToMap(wx, wy, x, y))
        throw std::runtime_error("Test point outside costmap");
      return map->getCost(x, y);
    };
    send(0);
    if (cell() != 0)
      throw std::runtime_error("free ground");
    if (!surface_only) {
      sensor_msgs::msg::PointCloud2 cloud;
      cloud.header.frame_id = "odom";
      sensor_msgs::PointCloud2Modifier modifier(cloud);
      modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
          "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
      modifier.resize(2);
      sensor_msgs::PointCloud2Iterator<float> px(cloud, "x"), py(cloud, "y"),
          pz(cloud, "z"), intensity(cloud, "intensity");
      *px = 1.f; *py = -.5f; *pz = -.2f; *intensity = .2f;
      ++px; ++py; ++pz; ++intensity;
      *px = -1.f; *py = -.5f; *pz = -.45f; *intensity = 0.f;
      cloud.header.stamp = n->now();
      cloud_pub->publish(cloud);
      publish_surface();
      if (cell(1., -.5) != 254 || cell(-1., -.5) != 0)
        throw std::runtime_error("Relative-height obstacle below map z=0 was lost or floor was marked");
      send(-1);
      modifier.resize(0);
      cloud.header.stamp = n->now();
      cloud_pub->publish(cloud);
      publish_surface();
      if (cell(1., -.5) != 0)
        throw std::runtime_error("Old intensity-layer obstacle remained after an empty observation");
      send(0);
      if (cell(1., -.5) != 0)
        throw std::runtime_error("Free ground retained an intensity obstacle");
    }
    paint(g, .05, -.20, .10, -.15, 100);
    paint(g, .5, 0., .6, .1, 100);
    publish_surface();
    if (cell(.075, -.175) == 254 || cell(.55, .05) != 254)
      throw std::runtime_error("Robot footprint and external obstacle were not separated");
    // Planning padding adds clearance; it must not erase real nearby returns.
    paint(g, .25, -.05, .30, .05, 100);
    publish_surface();
    if (cell(.275, .025) != 254)
      throw std::runtime_error("Planning padding erased an obstacle outside the physical footprint");
    paint(g, .25, -.05, .30, .05, 0);
    tf.transform.translation.x = .55;
    tf.transform.translation.y = .05;
    br.sendTransform(tf);
    publish_surface();
    if (cell(.55, .05) == 254)
      throw std::runtime_error("Current live obstacle remained inside the moved robot footprint");
    auto under_robot = surveyed;
    paint(under_robot, .5, 0., .6, .1, 100);
    under_robot.header.stamp = n->now();
    map_pub->publish(under_robot);
    publish_surface();
    if (cell(.55, .05) != 254)
      throw std::runtime_error("Footprint clearing erased a static map obstacle");
    tf.transform.translation.x = tf.transform.translation.y = 0.;
    br.sendTransform(tf);
    surveyed.header.stamp = n->now();
    map_pub->publish(surveyed);
    send(0);
    geometry_msgs::msg::PoseStamped start, goal;
    start.header.frame_id = goal.header.frame_id = "odom";
    start.pose.orientation.w = goal.pose.orientation.w = 1.;
    start.pose.position.x = -1.3;
    goal.pose.position.x = 1.3;
    auto plan = [&]() {
      return planner->createPlan(start, goal);
    };
    auto straight = plan();
    if (straight.poses.empty())
      throw std::runtime_error("Initial route missing");
    paint(g, -.2, -.4, .2, .4, 100);
    publish_surface();
    auto detour = plan();
    double deviation = 0.;
    for (const auto &p : detour.poses)
      deviation = std::max(deviation, std::abs(p.pose.position.y));
    if (detour.poses.empty() || deviation < .55)
      throw std::runtime_error("New obstacle did not produce a detour");
    std::cout << "New obstacle: route detour " << deviation << " m\n";
    send(0);
    auto restored = plan();
    for (const auto &p : restored.poses)
      if (std::abs(p.pose.position.y) > .15)
        throw std::runtime_error("Removed live obstacle still affects route");
    // A surveyed obstacle remains after live observations disappear.
    auto prior = surveyed;
    paint(prior, -.2, -.4, .2, .4, 100);
    prior.header.stamp = n->now();
    map_pub->publish(prior);
    send(-1);
    auto prior_detour = plan();
    deviation = 0.;
    for (const auto &p : prior_detour.poses)
      deviation = std::max(deviation, std::abs(p.pose.position.y));
    if (prior_detour.poses.empty() || deviation < .55)
      throw std::runtime_error("Surveyed obstacle did not produce a detour");
    // Unknown space across the complete map separates the route endpoints.
    paint(prior, -.2, -2., .2, 2., -1);
    prior.header.stamp = n->now();
    map_pub->publish(prior);
    send(-1);
    bool rejected = false;
    try {
      rejected = plan().poses.empty();
    } catch (const std::exception &) {
      rejected = true;
    }
    if (!rejected)
      throw std::runtime_error("Route crosses unobserved space");
    surveyed.header.stamp = n->now();
    map_pub->publish(surveyed);
    send(100);
    if (cell() != 254)
      throw std::runtime_error("obstacle");
    // One new observation replaces live costs, including unknown cells.
    auto publish_once = [&]() {
      g.header.stamp = n->now();
      pub->publish(g);
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    };
    std::fill(g.data.begin(), g.data.end(), -1);
    publish_once();
    if (cell() != 0)
      throw std::runtime_error("One new unknown observation retained an old live obstacle");
    paint(g, .9, -.1, 1.1, .1, 100);
    publish_surface();
    if (cell() != 254)
      throw std::runtime_error("Fresh obstacle missing before observation window moved");
    g.info.origin.position.x = -4.;
    std::fill(g.data.begin(), g.data.end(), -1);
    paint(g, -1., -.6, -.9, -.5, 100);
    publish_once();
    if (cell() != 0 || cell(-.95, -.55) != 254)
      throw std::runtime_error("Moved observation window retained old marks or lost new marks");
    g.info.origin.position.x = -2.;
    send(100);
    // ClearEntireCostmap discards the current frame; fresh frames rebuild it.
    {
      std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
          *cm->getCostmap()->getMutex());
      cm->resetLayers();
    }
    send(-1);
    if (cell() != 0)
      throw std::runtime_error("Recovery retained an old live obstacle");
    send(100);
    if (cell() != 254)
      throw std::runtime_error("Fresh observation did not mark an obstacle after reset");
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    if (cell() != 0 || cm->getLayeredCostmap()->isCurrent())
      throw std::runtime_error("Expired observation retained a live mark or remained current");
    send(0);
    if (cell() != 0)
      throw std::runtime_error("clearing");
    send(-1);
    if (cell() != 0)
      throw std::runtime_error("unobserved ground should retain surveyed free space");
    if (cell(1.05, 1.05) != 254 || cell(-.95, 1.05) < 253)
      throw std::runtime_error("surveyed obstacle or unknown map cell lost");
    for (int i = 0; i < 32; ++i) {
      const double angle = 2 * 3.141592653589793 * i / 32;
      if (cell(.2 * std::cos(angle), .2 * std::sin(angle)) != 0)
        throw std::runtime_error("live blind region covers robot footprint");
    }
    send(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    if (cm->getLayeredCostmap()->isCurrent())
      throw std::runtime_error("stale terrain");
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
    result = 1;
  }
  planner->deactivate();
  planner->cleanup();
  planner.reset();
  cm->deactivate();
  ex.cancel();
  thread.join();
  cm->cleanup();
  ex.remove_node(cm->get_node_base_interface());
  cm.reset();
  rclcpp::shutdown();
  return result;
}
