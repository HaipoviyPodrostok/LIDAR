#pragma once

#include <cassert>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "geometry/plane.hpp"
#include "geometry/vector_3d.hpp"

namespace lidar {

struct Point2D {
  double u = 0.0;
  double v = 0.0;

  [[nodiscard]] bool is_valid() const noexcept;
};

class RegionMask {
 public:
  explicit RegionMask(std::vector<Point2D> vertices) noexcept;

  [[nodiscard]] bool is_valid() const noexcept;
  [[nodiscard]] bool is_inside(const Point2D& pt) const noexcept;

 private:
  std::vector<Point2D> vertices_;
};

struct LidarConfig {
  double angle_min_deg = -30.0;
  double angle_step_deg = 0.25;
  double scan_time_step_sec = 0.02;
  double truck_speed_mm_s = 5000.0;

  [[nodiscard]] bool is_valid() const noexcept;
};

struct DepthView {
  std::span<const uint16_t> data;
  size_t rows;
  size_t cols;

  [[nodiscard]] const uint16_t& operator()(size_t r, size_t c) const noexcept {
    assert(r < rows && c < cols);
    return data[r * cols + c];
  }
};

[[nodiscard]] geometry::Vector3D get_3d_coords(
    const uint16_t depth, const double u, const double v,
    const LidarConfig& config) noexcept;

[[nodiscard]] std::vector<geometry::Vector3D> extract_cloud(
    const DepthView depth_map, const RegionMask& mask,
    const LidarConfig& config);

[[nodiscard]] std::optional<geometry::Plane> fit_plane_ransac(
    std::span<const geometry::Vector3D> cloud, size_t iterations = 2000,
    double threshold_mm = 30.0);

}  // namespace lidar