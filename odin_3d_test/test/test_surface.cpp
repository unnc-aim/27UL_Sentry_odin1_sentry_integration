#include "surface.hpp"
#include <iostream>
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
pcl::PointCloud<pcl::PointXYZ> plane(double slope) {
  pcl::PointCloud<pcl::PointXYZ> p;
  for (int y = 0; y < 100; y++)
    for (int x = 0; x < 100; x++) {
      float xx = (x + .5) * .02, yy = (y + .5) * .02;
      p.push_back({xx, yy, float(xx * slope)});
    }
  return p;
}
int main() {
  odin3d::Settings s;
  auto flat = plane(0);
  auto g = odin3d::classify(flat, 0, 0, 20, 20, s, .5, .5, 0);
  require(g.cost[210] == 0, "flat support");
  auto ramp = odin3d::classify(plane(std::tan(20 * M_PI / 180)), 0, 0, 20, 20,
                               s, .5, .5, .5 * std::tan(20 * M_PI / 180));
  require(ramp.cost[210] > 0 && ramp.cost[210] < 100, "20 degree ramp");
  auto steep = odin3d::classify(plane(1), 0, 0, 20, 20, s, .5, .5, .5);
  require(steep.cost[210] == 100, "steep slope");
  for (int j = 0; j < 20; j++)
    flat.push_back({1.05f, 1.05f, .40f});
  g = odin3d::classify(flat, 0, 0, 20, 20, s, .5, .5, 0);
  require(g.cost[210] == 100, "low overhead");
  auto hole = plane(0);
  hole.erase(std::remove_if(hole.begin(), hole.end(),
                            [](auto p) {
                              return p.x > .8 && p.x < 1.2 && p.y > .8 &&
                                     p.y < 1.2;
                            }),
             hole.end());
  g = odin3d::classify(hole, 0, 0, 20, 20, s, .5, .5, 0);
  require(g.cost[210] == -1, "hole stays unknown");
  auto clear = odin3d::classify(plane(0), 0, 0, 20, 20, s, .5, .5, 0);
  require(clear.cost[210] == 0, "removed object clears with observed ground");
  auto elevated = plane(0);
  for (auto &p : elevated)
    p.z = .3;
  auto unsupported = odin3d::classify(elevated, 0, 0, 20, 20, s, .5, .5, 0);
  require(unsupported.cost[210] == -1,
          "raised platform is not connected floor");
  auto noisy = plane(0);
  for (int j = 0; j < 4; j++)
    noisy.push_back({1.05f, 1.05f, .12f});
  auto static_settings = s;
  static_settings.obstacle_support = 8;
  auto clean =
      odin3d::classify(noisy, 0, 0, 20, 20, static_settings, .5, .5, 0);
  require(clean.cost[210] == 0, "sparse historical returns in static map");
  for (int j = 0; j < 20; j++)
    noisy.push_back({1.05f, 1.05f, .20f});
  auto object =
      odin3d::classify(noisy, 0, 0, 20, 20, static_settings, .5, .5, 0);
  require(object.cost[210] == 100, "supported static obstacle retained");
  auto floors = plane(0);
  auto flat_settings = s;
  flat_settings.flat_ground = true;
  auto evaluate = [&](float roof, double resolution = .1) {
    auto data = floors;
    for (auto p : plane(0)) {
      p.z = roof;
      data.push_back(p);
    }
    auto settings = flat_settings;
    settings.resolution = resolution;
    const int size = std::lround(2. / resolution);
    return odin3d::classify(data, 0, 0, size, size, settings, .5, .5, 0);
  };
  require(evaluate(2.).cost[210] == 0, "ceiling above measured floor");
  require(evaluate(.60).cost[210] == 0, "60cm overhead clearance");
  require(evaluate(.45).cost[210] == 100, "45cm overhead collision");
  const int fine_center = 21 * 40 + 21;
  require(evaluate(2., .05).cost[fine_center] == 0, "5cm grid floor support");
  require(evaluate(.60, .05).cost[fine_center] == 0, "5cm grid 60cm opening");
  require(evaluate(.45, .05).cost[fine_center] == 100, "5cm grid 45cm opening");
  auto ceiling_only = plane(0);
  for (auto &p : ceiling_only)
    p.z = 2.;
  require(odin3d::classify(ceiling_only, 0, 0, 20, 20, flat_settings, .5, .5, 0)
                  .cost[210] == -1,
          "ceiling alone is not floor");
  auto gap = hole;
  for (auto p : plane(0)) {
    p.z = 2.;
    gap.push_back(p);
  }
  require(
      odin3d::classify(gap, 0, 0, 20, 20, flat_settings, .5, .5, 0).cost[210] ==
          -1,
      "ceiling does not fill floor hole");
  // A map containing one measured point per 5 cm cell still describes floor.
  pcl::PointCloud<pcl::PointXYZ> sparse_floor;
  for (int y = 0; y < 40; ++y)
    for (int x = 0; x < 40; ++x)
      sparse_floor.push_back({float((x + .5) * .05),
                              float((y + .5) * .05), 0.f});
  auto native_settings = flat_settings;
  native_settings.resolution = .05;
  native_settings.min_points = 1;
  native_settings.obstacle_support = 8;
  auto native = odin3d::classify(sparse_floor, 0, 0, 40, 40,
                                native_settings, .5, .5, 0);
  require(std::all_of(native.cost.begin(), native.cost.end(),
                      [](auto cost) { return cost == 0; }),
          "single observed floor point supports a static cell");
  sparse_floor.erase(std::remove_if(sparse_floor.begin(), sparse_floor.end(),
      [](auto p) { return p.x > .8 && p.x < 1.2 && p.y > .8 && p.y < 1.2; }),
      sparse_floor.end());
  for (int j = 0; j < 8; ++j) {
    sparse_floor.push_back({1.575f, 1.575f, .45f});
    sparse_floor.push_back({1.375f, 1.375f, .60f});
    sparse_floor.push_back({1.075f, 1.075f, 2.f});
  }
  native = odin3d::classify(sparse_floor, 0, 0, 40, 40,
                           native_settings, .5, .5, 0);
  require(native.cost[21 * 40 + 21] == -1,
          "unobserved floor stays unknown even under a ceiling");
  require(native.cost[31 * 40 + 31] == 100,
          "45cm structure remains occupied with sparse floor");
  require(native.cost[27 * 40 + 27] == 0,
          "60cm structure retains measured floor clearance");
  odin3d::Surface gaps{12, 12, .05, 0, 0, std::vector<int8_t>(144, 0), {}, {}};
  gaps.cost[2 * 12 + 2] = -1;
  gaps.cost[3 * 12 + 3] = -1;
  gaps.cost[5 * 12 + 5] = -1;
  gaps.cost[5 * 12 + 6] = 100;
  gaps.cost[0] = -1;
  for (int y = 8; y < 11; ++y)
    for (int x = 8; x < 11; ++x)
      gaps.cost[y * 12 + x] = -1;
  odin3d::closeGroundGaps(gaps, 2);
  require(gaps.cost[2 * 12 + 2] == 0 && gaps.cost[3 * 12 + 3] == 0,
          "10cm enclosed floor sampling gaps close");
  require(gaps.cost[5 * 12 + 5] == -1 && gaps.cost[5 * 12 + 6] == 100,
          "obstacle edge and obstacle remain unchanged");
  require(gaps.cost[0] == -1 && gaps.cost[9 * 12 + 9] == -1,
          "survey boundary and larger gaps stay unknown");
  // Require support inside the vehicle's height interval, independently of
  // the dense valid beam surface above it.
  auto arch_cloud = plane(0);
  for (auto p : plane(0)) {p.z = .60; arch_cloud.push_back(p);}
  arch_cloud.push_back({1.075f, 1.075f, .53f});
  auto live = flat_settings;
  live.resolution = .05;
  live.obstacle_support = 3;
  live.clearance_support = 6;
  auto arch = odin3d::classify(arch_cloud, 0, 0, 40, 40, live, .5, .5, 0);
  require(arch.cost[21 * 40 + 21] == 0,
          "single low return at a 60cm beam edge stays unmarked");
  for (int i = 0; i < 3; ++i)
    arch_cloud.push_back({1.275f, 1.075f, .15f});
  arch = odin3d::classify(arch_cloud, 0, 0, 40, 40, live, .5, .5, 0);
  require(arch.cost[25 * 40 + 21] == 0 && arch.cost[21 * 40 + 25] == 100,
          "supported 15cm object retains its own occupied cell");
  for (int i = 0; i < 5; ++i)
    arch_cloud.push_back({1.075f, 1.075f, .53f});
  arch = odin3d::classify(arch_cloud, 0, 0, 40, 40, live, .5, .5, 0);
  require(arch.cost[21 * 40 + 21] == 100,
          "six supported returns retain a structure in the top clearance band");
  auto low_roof = plane(0);
  for (auto p : plane(0)) {p.z = .45; low_roof.push_back(p);}
  auto low_arch = odin3d::classify(low_roof, 0, 0, 40, 40, live, .5, .5, 0);
  require(low_arch.cost[21 * 40 + 21] == 100, "45cm beam remains an obstacle");
  std::cout << "flat, ramp, steep slope, overhead, hole and clearing passed\n";
}
