#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <queue>
#include <stdexcept>
#include <vector>

namespace odin3d {
struct Settings {
  double resolution = 0.10, height = 0.50, clearance = 0.05, slope = 30.0;
  double obstacle_height = 0.10, step = 0.03, roughness = 0.04,
         normal_radius = 0.25;
  bool flat_ground = false;
  double max_ground_gap = 0.0;
  int min_points = 3;
  int obstacle_support = 1;
  int clearance_support = 0;  // Zero reuses obstacle_support for the extra height band.
  void validate() const {
    for (double v : {resolution, height, clearance, slope, obstacle_height,
                     roughness, normal_radius})
      if (!std::isfinite(v) || v <= 0)
        throw std::invalid_argument(
            "Positive finite surface settings required");
    if (slope >= 90 || clearance >= height || !std::isfinite(step) ||
        step < 0 || min_points < 1 || obstacle_support < 1 ||
        !std::isfinite(max_ground_gap) || max_ground_gap < 0 ||
        clearance_support < 0 ||
        (clearance_support > 0 && clearance_support < obstacle_support))
      throw std::invalid_argument("Invalid vehicle geometry");
  }
};
struct Surface {
  int width, height;
  double resolution, x, y;
  std::vector<int8_t> cost;
  std::vector<float> ground, slope;
};

// Close only small, enclosed sampling gaps on a confirmed flat floor.
// Obstacle-adjacent gaps and the survey boundary retain their original state.
inline void closeGroundGaps(Surface &out, int span) {
  if (span < 1)
    return;
  const int w = out.width, h = out.height;
  std::vector<bool> seen(out.cost.size(), false);
  for (int seed = 0; seed < int(out.cost.size()); ++seed) {
    if (seen[seed] || out.cost[seed] != -1)
      continue;
    std::vector<int> component{seed};
    seen[seed] = true;
    bool enclosed = true;
    int xmin = seed % w, xmax = xmin, ymin = seed / w, ymax = ymin;
    for (size_t k = 0; k < component.size(); ++k) {
      const int x = component[k] % w, y = component[k] / w;
      xmin = std::min(xmin, x); xmax = std::max(xmax, x);
      ymin = std::min(ymin, y); ymax = std::max(ymax, y);
      for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
          const int nx = x + dx, ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
            enclosed = false;
            continue;
          }
          const int j = ny * w + nx;
          if (out.cost[j] >= 100)
            enclosed = false;
          if (out.cost[j] == -1 && !seen[j]) {
            seen[j] = true;
            component.push_back(j);
          }
        }
    }
    if (enclosed && xmax - xmin + 1 <= span && ymax - ymin + 1 <= span)
      for (int i : component)
        out.cost[i] = 0;
  }
}
// Reuse PCL's least-squares normal estimator on measured ground cells.
inline Surface classify(const pcl::PointCloud<pcl::PointXYZ> &cloud, double ox,
                        double oy, int w, int h, const Settings &s,
                        double robot_x, double robot_y, double robot_z,
                        bool connected = true, double ground_dx = 0,
                        double ground_dy = 0) {
  s.validate();
  if (w < 1 || h < 1 || static_cast<size_t>(w) * h > 4000000)
    throw std::invalid_argument("Surface dimensions out of range");
  const int n = w * h;
  const float nan = std::numeric_limits<float>::quiet_NaN();
  Surface out{w,
              h,
              s.resolution,
              ox,
              oy,
              std::vector<int8_t>(n, -1),
              std::vector<float>(n, nan),
              std::vector<float>(n, nan)};
  std::vector<std::vector<float>> cells(n);
  for (const auto &p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    int x = std::floor((p.x - ox) / s.resolution),
        y = std::floor((p.y - oy) / s.resolution);
    if (x >= 0 && x < w && y >= 0 && y < h)
      cells[y * w + x].push_back(p.z);
  }
  if (s.flat_ground) {
    auto candidates = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    for (const auto &p : cloud)
      if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
          std::abs(p.z - robot_z) < .25)
        candidates->push_back(p);
    if (candidates->size() < 30)
      return out;
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(candidates);
    voxel.setLeafSize(.05f, .05f, .05f);
    auto sampled = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    voxel.filter(*sampled);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(10 * M_PI / 180.0);
    segmentation.setDistanceThreshold(.025);
    segmentation.setMaxIterations(300);
    segmentation.setInputCloud(sampled);
    pcl::PointIndices inliers;
    pcl::ModelCoefficients plane;
    segmentation.segment(inliers, plane);
    if (plane.values.size() != 4 || inliers.indices.size() < 30 ||
        inliers.indices.size() < sampled->size() * .25)
      return out;
    auto a = plane.values[0], b = plane.values[1], c = plane.values[2],
         d = plane.values[3];
    if (!std::isfinite(c) || std::abs(c) < .9)
      return out;
    double angle = std::acos(std::clamp(
        std::abs(double(c)) / std::sqrt(a * a + b * b + c * c), 0., 1.));
    for (int i = 0; i < n; i++) {
      double x = ox + (i % w + .5) * s.resolution,
             y = oy + (i / w + .5) * s.resolution;
      double floor = -(a * x + b * y + d) / c;
      int support = 0, obstacles = 0, body_obstacles = 0;
      for (float z : cells[i]) {
        double dz = z - floor;
        if (std::abs(dz) <= .05)
          support++;
        if (dz > s.obstacle_height && dz <= s.height + s.clearance)
          obstacles++;
        if (dz > s.obstacle_height && dz <= s.height)
          body_obstacles++;
      }
      out.ground[i] = floor;
      out.slope[i] = angle;
      const int margin_support = s.clearance_support > 0
                                     ? s.clearance_support : s.obstacle_support;
      if (body_obstacles >= s.obstacle_support || obstacles >= margin_support)
        out.cost[i] = 100;
      else if (support >= s.min_points)
        out.cost[i] = 0;
    }
    closeGroundGaps(out, std::floor(s.max_ground_gap / s.resolution + 1e-6));
    // Retain only floor connected to the measured starting area.
    if (connected) {
      std::vector<bool> reached(n, false);
      std::queue<int> todo;
      for (int i = 0; i < n; i++)
        if (out.cost[i] == 0 &&
            std::hypot(ox + (i % w + .5) * s.resolution - robot_x,
                       oy + (i / w + .5) * s.resolution - robot_y) < .65) {
          reached[i] = true;
          todo.push(i);
        }
      while (!todo.empty()) {
        int i = todo.front();
        todo.pop();
        for (auto [dx, dy] : std::vector<std::pair<int, int>>{
                 {1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
          int x = i % w + dx, y = i / w + dy;
          if (x < 0 || x >= w || y < 0 || y >= h)
            continue;
          int j = y * w + x;
          if (out.cost[j] == 0 && !reached[j]) {
            reached[j] = true;
            todo.push(j);
          }
        }
      }
      for (int i = 0; i < n; i++)
        if (out.cost[i] == 0 && !reached[i])
          out.cost[i] = -1;
    }
    return out;
  }
  auto ground = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  std::vector<int> indices;
  for (int i = 0; i < n; i++) {
    auto &z = cells[i];
    if (static_cast<int>(z.size()) < s.min_points)
      continue;
    std::sort(z.begin(), z.end());
    out.ground[i] = z[static_cast<size_t>((z.size() - 1) * 0.1)];
    ground->push_back(pcl::PointXYZ(ox + (i % w + .5) * s.resolution,
                                    oy + (i / w + .5) * s.resolution,
                                    out.ground[i]));
    indices.push_back(i);
  }
  if (ground->size() < 6)
    return out;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estimator;
  estimator.setInputCloud(ground);
  estimator.setRadiusSearch(s.normal_radius);
  estimator.setViewPoint(robot_x, robot_y, robot_z + 10);
  pcl::PointCloud<pcl::Normal> normals;
  estimator.compute(normals);
  for (size_t k = 0; k < indices.size(); k++) {
    int i = indices[k];
    auto normal = normals[k];
    if (!std::isfinite(normal.normal_z) || std::abs(normal.normal_z) < 1e-6)
      continue;
    double angle =
        std::acos(std::clamp(std::abs(double(normal.normal_z)), 0.0, 1.0));
    out.slope[i] = angle;
    if (angle > s.slope * M_PI / 180.0 || normal.curvature > s.roughness) {
      out.cost[i] = 100;
      continue;
    }
    // Ground-normal tolerance accounts for the height variation inside a cell.
    double floor_band =
        s.obstacle_height + std::tan(angle) * s.resolution * std::sqrt(2.0);
    int obstacle_points = 0;
    for (float z : cells[i]) {
      double dz = z - out.ground[i];
      if (dz > floor_band && dz <= s.height + s.clearance) {
        ++obstacle_points;
      }
    }
    out.cost[i] = obstacle_points >= s.obstacle_support
                      ? 100
                      : static_cast<int8_t>(std::min(
                            70.0, 70.0 * angle / (s.slope * M_PI / 180.0)));
  }
  if (connected) {
    std::vector<bool> reached(n, false);
    std::queue<int> q;
    // Seeds require measured ground close to the chassis ground plane.
    for (int i : indices) {
      double x = ox + (i % w + .5) * s.resolution,
             y = oy + (i / w + .5) * s.resolution;
      if (out.cost[i] >= 0 && out.cost[i] < 100 &&
          std::hypot(x - robot_x, y - robot_y) < 0.65 &&
          std::abs(out.ground[i] - (robot_z + ground_dx * (x - robot_x) +
                                    ground_dy * (y - robot_y))) <=
              s.step + 0.02) {
        reached[i] = true;
        q.push(i);
      }
    }
    while (!q.empty()) {
      int i = q.front();
      q.pop();
      for (auto [dx, dy] :
           std::vector<std::pair<int, int>>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}}) {
        int x = i % w + dx, y = i / w + dy;
        if (x < 0 || x >= w || y < 0 || y >= h)
          continue;
        int j = y * w + x;
        if (reached[j] || out.cost[j] < 0 || out.cost[j] >= 100)
          continue;
        // Account for smooth slope; reject abrupt changes beyond the fitted
        // surface.
        double predicted = 0;
        if (std::isfinite(out.slope[i]) && std::isfinite(out.slope[j]))
          predicted =
              s.resolution * std::tan(std::min(out.slope[i], out.slope[j]));
        if (std::abs(out.ground[i] - out.ground[j]) > s.step + predicted)
          continue;
        reached[j] = true;
        q.push(j);
      }
    }
    for (int i : indices)
      if (!reached[i] && out.cost[i] >= 0 && out.cost[i] < 100)
        out.cost[i] = -1;
  }
  return out;
}
} // namespace odin3d
