#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <nlohmann/json.hpp>

#include "lidar/lidar_processor.hpp"
#include "stb_image.h"
#include "stb_image_write.h"

using json = nlohmann::json;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <depth_map.png> <mask.json>\n";
    return 1;
  }

  std::string png_path = argv[1];
  std::string json_path = argv[2];

  std::ifstream f(json_path);
  if (!f.is_open()) {
    std::cerr << "Failed to open json file: " << json_path << "\n";
    return 1;
  }
  json data = json::parse(f);

  std::vector<lidar::Point2D> polygon;
  if (data.contains("objects") && data["objects"].is_array() &&
      !data["objects"].empty()) {
    const auto& obj = data["objects"][0];
    if (obj.contains("data") && obj["data"].is_array()) {
      for (const auto& point_arr : obj["data"]) {
        if (point_arr.is_array() && point_arr.size() >= 2) {
          polygon.push_back(
              {point_arr[0].get<double>(), point_arr[1].get<double>()});
        }
      }
    }
  }

  lidar::RegionMask mask(polygon);
  if (!mask.is_valid()) {
    std::cerr << "Invalid mask\n";
    return 1;
  }

  int width, height, channels;
  uint16_t* img_data =
      stbi_load_16(png_path.c_str(), &width, &height, &channels, 1);
  if (!img_data) {
    std::cerr << "Failed to load image: " << png_path << "\n";
    return 1;
  }

  lidar::DepthView depth_map{
      std::span<const uint16_t>(img_data,
                                static_cast<size_t>(width) *
                                    static_cast<size_t>(height)),
      static_cast<size_t>(height), static_cast<size_t>(width)};

  lidar::LidarConfig config;
  constexpr double threshold_mm = 30.0;

  auto cloud = lidar::extract_cloud(depth_map, mask, config);

  std::cout << "Extracted " << cloud.size()
            << " valid points inside the mask.\n";

  auto plane_opt = lidar::fit_plane_ransac(cloud, 2000, threshold_mm);

  if (plane_opt) {
    const auto& p = *plane_opt;
    std::cout << "Found plane:\n"
              << "Point: [" << p.get_r().x << ", " << p.get_r().y << ", "
              << p.get_r().z << "]\n"
              << "Normal: [" << p.get_normal().x << ", " << p.get_normal().y
              << ", " << p.get_normal().z << "]\n";
  } else {
    std::cout << "Failed to find plane.\n";
    stbi_image_free(img_data);
    return 1;
  }

  // --- Генерация карты отклонений ---
  const auto& plane = *plane_opt;
  const size_t w = static_cast<size_t>(width);
  const size_t h = static_cast<size_t>(height);

  std::vector<uint8_t> rgb(w * h * 3, 0);

  for (size_t r = 0; r < h; ++r) {
    for (size_t c = 0; c < w; ++c) {
      const size_t pixel = (r * w + c) * 3;

      if (!mask.is_inside({static_cast<double>(c), static_cast<double>(r)})) {
        // Вне маски — тёмно-серый фон
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

      auto pt3d = lidar::get_3d_coords(depth, static_cast<double>(c),
                                        static_cast<double>(r), config);
      double dist_val =
          std::abs((pt3d - plane.get_r()).scalar(plane.get_normal()));

      if (dist_val <= threshold_mm) {
        // Инлайер (дно кузова) — серый, яркость зависит от близости к плоскости
        auto gray = static_cast<uint8_t>(
            128 + std::clamp(static_cast<int>(64.0 * (1.0 - dist_val / threshold_mm)),
                             0, 64));
        rgb[pixel + 0] = gray;
        rgb[pixel + 1] = gray;
        rgb[pixel + 2] = gray;
      } else {
        // Выброс (снег, пыль) — красный, интенсивность по отклонению
        double intensity = std::clamp(dist_val / (threshold_mm * 5.0), 0.0, 1.0);
        rgb[pixel + 0] = static_cast<uint8_t>(128 + 127 * intensity);
        rgb[pixel + 1] = 0;
        rgb[pixel + 2] = 0;
      }
    }
  }

  stbi_image_free(img_data);

  std::string out_path = "deviation_map.png";
  stbi_write_png(out_path.c_str(), width, height, 3, rgb.data(),
                 width * 3);
  std::cout << "Deviation map saved to: " << out_path << "\n";

  return 0;
}
