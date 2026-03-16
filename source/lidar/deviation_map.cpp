#include "lidar/deviation_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "third_party/stb_image_write.h"

namespace lidar {

std::string render_deviation_map(const DepthView& depth_map,
                                 const RegionMask& mask,
                                 const LidarConfig& config,
                                 const geometry::Plane& plane,
                                 double threshold_mm,
                                 const std::string& output_name) {
  const size_t w = depth_map.cols;
  const size_t h = depth_map.rows;

  std::vector<uint8_t> rgb(w * h * 3, 0);

  for (size_t r = 0; r < h; ++r) {
    for (size_t c = 0; c < w; ++c) {
      const size_t pixel = (r * w + c) * 3;

      if (!mask.is_inside({static_cast<double>(c), static_cast<double>(r)})) {
        rgb[pixel + 0] = 30;
        rgb[pixel + 1] = 30;
        rgb[pixel + 2] = 30;
        continue;
      }

      uint16_t depth = depth_map(r, c);
      if (depth == 0) {
        rgb[pixel + 0] = 0;
        rgb[pixel + 1] = 0;
        rgb[pixel + 2] = 0;
        continue;
      }

      auto pt3d = get_3d_coords(depth, static_cast<double>(c),
                                static_cast<double>(r), config);
      double dist_val =
          std::abs((pt3d - plane.get_r()).scalar(plane.get_normal()));

      if (dist_val <= threshold_mm) {
        auto gray = static_cast<uint8_t>(
            128 +
            std::clamp(static_cast<int>(64.0 * (1.0 - dist_val / threshold_mm)),
                       0, 64));
        rgb[pixel + 0] = gray;
        rgb[pixel + 1] = gray;
        rgb[pixel + 2] = gray;
      } else {
        double intensity =
            std::clamp(dist_val / (threshold_mm * 5.0), 0.0, 1.0);
        rgb[pixel + 0] = static_cast<uint8_t>(128 + 127 * intensity);
        rgb[pixel + 1] = 0;
        rgb[pixel + 2] = 0;
      }
    }
  }

  namespace fs = std::filesystem;
  fs::path output_dir = fs::path("assets") / "output";
  fs::create_directories(output_dir);

  fs::path out_path = output_dir / (output_name + ".png");
  stbi_write_png(out_path.c_str(), static_cast<int>(w), static_cast<int>(h), 3,
                 rgb.data(), static_cast<int>(w) * 3);

  return out_path.string();
}

}  // namespace lidar
