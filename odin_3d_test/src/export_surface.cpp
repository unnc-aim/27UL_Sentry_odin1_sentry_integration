#include "surface.hpp"
#include <filesystem>
#include <fstream>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
int main(int argc, char **argv) {
  if (argc < 7 || argc > 11) {
    std::cerr << "Usage: export_surface INPUT.ply OUTPUT_DIR SEED_X SEED_Y "
                 "SEED_GROUND_Z RESOLUTION [OBSTACLE_SUPPORT=8] [flat|local] "
                 "[GROUND_SUPPORT=1] [FLAT_GROUND_GAP=0.10]\n";
    return 2;
  }
  try {
    std::filesystem::path file(argv[1]), dir(argv[2]);
    if (std::filesystem::exists(dir))
      throw std::runtime_error("Choose a new output directory");
    pcl::PointCloud<pcl::PointXYZ> cloud;
    int rc = file.extension() == ".pcd"
                 ? pcl::io::loadPCDFile(file.string(), cloud)
                 : pcl::io::loadPLYFile(file.string(), cloud);
    if (rc < 0 || cloud.empty())
      throw std::runtime_error("Read a nonempty PLY or PCD map");
    pcl::PointXYZ lo, hi;
    pcl::getMinMax3D(cloud, lo, hi);
    odin3d::Settings s;
    s.resolution = std::stod(argv[6]);
    s.obstacle_support = argc >= 8 ? std::stoi(argv[7]) : 8;
    s.flat_ground = argc < 9 || std::string(argv[8]) == "flat";
    // Native maps already merge observations spatially. One measured floor
    // point supports its static cell; live terrain retains its own threshold.
    s.min_points = argc >= 10 ? std::stoi(argv[9]) : 1;
    s.max_ground_gap = argc >= 11 ? std::stod(argv[10]) : 0.10;
    if (argc >= 9 && std::string(argv[8]) != "flat" &&
        std::string(argv[8]) != "local")
      throw std::invalid_argument("Use flat or local ground mode");
    s.validate();
    int w = std::ceil((hi.x - lo.x) / s.resolution) + 1,
        h = std::ceil((hi.y - lo.y) / s.resolution) + 1;
    auto grid = odin3d::classify(cloud, lo.x, lo.y, w, h, s, std::stod(argv[3]),
                                 std::stod(argv[4]), std::stod(argv[5]));
    if (std::none_of(grid.cost.begin(), grid.cost.end(),
                     [](auto c) { return c >= 0 && c < 100; }))
      throw std::runtime_error("No connected ground: check seed and map axes");
    std::filesystem::create_directories(dir);
    std::ofstream image(dir / "navigation.pgm", std::ios::binary);
    image << "P5\n" << w << " " << h << "\n255\n";
    for (int y = h - 1; y >= 0; y--)
      for (int x = 0; x < w; x++) {
        auto c = grid.cost[y * w + x];
        unsigned char v = c < 0 ? 205 : c >= 100 ? 0 : 254;
        image.write(reinterpret_cast<char *>(&v), 1);
      }
    std::ofstream yaml(dir / "navigation.yaml");
    yaml << "image: navigation.pgm\nresolution: " << s.resolution
         << "\norigin: [" << lo.x << ", " << lo.y
         << ", 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
    std::ofstream heights(dir / "terrain.csv");
    heights << "x,y,ground_z,slope_rad,cost\n";
    for (int i = 0; i < w * h; i++)
      heights << lo.x + (i % w + .5) * s.resolution << ","
              << lo.y + (i / w + .5) * s.resolution << "," << grid.ground[i]
              << "," << grid.slope[i] << "," << int(grid.cost[i]) << "\n";
    std::filesystem::copy_file(
        file, dir / ("source_map" + file.extension().string()));
    std::cout << "Saved geometry map and original input (including original "
                 "color fields): "
              << dir << "\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
}
