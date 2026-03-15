#include "lidar/lidar_processor.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <random>

#include "math/math.hpp"

namespace lidar {

bool Point2D::is_valid() const noexcept {
  return true;
}

RegionMask::RegionMask(std::vector<Point2D> vertices) noexcept
    : vertices_(std::move(vertices)) {}

bool RegionMask::is_valid() const noexcept {
  return vertices_.size() >= 3;
}

bool RegionMask::is_inside(const Point2D& pt) const noexcept {
  if (!is_valid()) { return false; }

  bool inside = false;

  for (size_t i = 0, j = vertices_.size() - 1; i < vertices_.size(); j = i++) {
    const auto& vi = vertices_[i];
    const auto& vj = vertices_[j];

    if (((vi.v > pt.v) != (vj.v > pt.v)) &&
        (pt.u < (vj.u - vi.u) * (pt.v - vi.v) / (vj.v - vi.v) + vi.u)) {
      inside = !inside;
    }
  }
  return inside;
}

bool LidarConfig::is_valid() const noexcept {
  return angle_step_deg > 0.0 && scan_time_step_sec > 0.0 &&
         truck_speed_mm_s > 0.0;
}

geometry::Vector3D get_3d_coords(uint16_t depth, double u, double v,
                                 const LidarConfig& config) noexcept {
  if (depth == 0) { return {0.0, 0.0, 0.0}; }
  const double angle_deg = config.angle_min_deg + u * config.angle_step_deg;
  const double angle_rad = angle_deg * std::numbers::pi / 180.0;

  const double d = static_cast<double>(depth);
  const double x = d * std::sin(angle_rad);
  const double y = v * config.scan_time_step_sec * config.truck_speed_mm_s;
  const double z = d * std::cos(angle_rad);

  return {x, y, z};
}

std::vector<geometry::Vector3D> extract_cloud(
    DepthView depth_map, const RegionMask& mask,
    const LidarConfig& config) noexcept {
  std::vector<geometry::Vector3D> cloud;
  cloud.reserve(depth_map.rows * depth_map.cols / 2);

  for (size_t r = 0; r < depth_map.rows; ++r) {
    for (size_t c = 0; c < depth_map.cols; ++c) {
      if (mask.is_valid() &&
          !mask.is_inside({static_cast<double>(c), static_cast<double>(r)})) {
        continue;
      }

      uint16_t depth = depth_map(r, c);
      if (depth == 0) { continue; }

      cloud.push_back(get_3d_coords(depth, c, r, config));
    }
  }
  cloud.shrink_to_fit();
  return cloud;
}

std::optional<geometry::Plane> fit_plane_ransac(
    std::span<const geometry::Vector3D> cloud, size_t iterations,
    double threshold_mm) noexcept {
  if (cloud.size() < 3) { return std::nullopt; }

  std::mt19937 gen(42);
  std::uniform_int_distribution<size_t> dist(0, cloud.size() - 1);

  std::optional<geometry::Plane> best_plane;
  size_t max_inliers = 0;

  for (size_t i = 0; i < iterations; ++i) {
    size_t idx1 = dist(gen);
    size_t idx2 = dist(gen);
    size_t idx3 = dist(gen);

    if (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) { continue; }

    const auto& p1 = cloud[idx1];
    const auto& p2 = cloud[idx2];
    const auto& p3 = cloud[idx3];

    auto v1 = p2 - p1;
    auto v2 = p3 - p1;
    auto normal = v1.cross(v2);

    double len = normal.length();
    if (len < math::eps) { continue; }

    geometry::Plane plane(p1, normal);
    if (!plane.is_valid()) { continue; }

    size_t inliers = 0;
    for (const auto& p : cloud) {
      double dist_val =
          std::abs((p - plane.get_r()).scalar(plane.get_normal()));
      if (dist_val <= threshold_mm) { inliers++; }
    }

    if (inliers > max_inliers) {
      max_inliers = inliers;
      best_plane = plane;
    }
  }

  if (!best_plane) { return std::nullopt; }

  // --- Least Squares Refinement ---
  // Собираем инлайеры лучшей плоскости RANSAC
  std::vector<geometry::Vector3D> inliers_vec;
  inliers_vec.reserve(max_inliers);
  for (const auto& p : cloud) {
    double dist_val =
        std::abs((p - best_plane->get_r()).scalar(best_plane->get_normal()));
    if (dist_val <= threshold_mm) { inliers_vec.push_back(p); }
  }

  if (inliers_vec.size() < 3) { return best_plane; }

  // Центроид (среднее арифметическое)
  geometry::Vector3D centroid{0.0, 0.0, 0.0};
  for (const auto& p : inliers_vec) {
    centroid = centroid + p;
  }
  const double n_inv = 1.0 / static_cast<double>(inliers_vec.size());
  centroid = centroid * n_inv;

  // Ковариационная матрица 3x3 (симметричная)
  // | cxx cxy cxz |
  // | cxy cyy cyz |
  // | cxz cyz czz |
  double cxx = 0.0, cxy = 0.0, cxz = 0.0;
  double cyy = 0.0, cyz = 0.0, czz = 0.0;
  for (const auto& p : inliers_vec) {
    const double dx = p.x - centroid.x;
    const double dy = p.y - centroid.y;
    const double dz = p.z - centroid.z;
    cxx += dx * dx;
    cxy += dx * dy;
    cxz += dx * dz;
    cyy += dy * dy;
    cyz += dy * dz;
    czz += dz * dz;
  }

  // Собственный вектор, соответствующий наименьшему собственному значению
  // ковариационной матрицы, является нормалью МНК-плоскости.
  // Для симметричной 3x3 матрицы решаем характеристическое уравнение
  // det(C - λI) = 0 аналитически (формула Кардано).

  // Коэффициенты характеристического полинома: -λ^3 + c1*λ^2 - c2*λ + c3 = 0
  const double c1 = cxx + cyy + czz;  // trace
  const double c2 = cxx * cyy + cxx * czz + cyy * czz -
                     cxy * cxy - cxz * cxz - cyz * cyz;
  const double c3 = cxx * cyy * czz + 2.0 * cxy * cyz * cxz -
                     cxx * cyz * cyz - cyy * cxz * cxz - czz * cxy * cxy;

  // Приведение к депрессивному кубическому: t^3 + pt + q = 0
  // где λ = t + c1/3
  const double c1_3 = c1 / 3.0;
  const double p = c2 - c1 * c1_3;
  const double q = c3 - c1_3 * c2 + 2.0 * c1_3 * c1_3 * c1_3;

  // Все три собственных значения вещественные (матрица симметрична)
  // Используем тригонометрическую формулу Кардано
  const double p_3 = p / 3.0;
  const double q_2 = q / 2.0;
  const double discriminant = q_2 * q_2 + p_3 * p_3 * p_3;

  double lambda_min = 0.0;
  if (discriminant < 0.0) {
    const double r_val = std::sqrt(-p_3 * p_3 * p_3);
    const double theta = std::acos(std::clamp(-q_2 / r_val, -1.0, 1.0)) / 3.0;
    const double two_sqrt_neg_p3 = 2.0 * std::sqrt(-p_3);

    const double l1 = two_sqrt_neg_p3 * std::cos(theta) + c1_3;
    const double l2 =
        two_sqrt_neg_p3 * std::cos(theta + 2.0 * std::numbers::pi / 3.0) +
        c1_3;
    const double l3 =
        two_sqrt_neg_p3 * std::cos(theta + 4.0 * std::numbers::pi / 3.0) +
        c1_3;

    lambda_min = std::min({l1, l2, l3});
  } else {
    // Вырожденный случай: используем плоскость RANSAC без уточнения
    return best_plane;
  }

  // Находим собственный вектор для lambda_min: (C - lambda_min * I) * v = 0
  // Решаем через кросс-произведение двух строк матрицы (C - λI)
  const double a00 = cxx - lambda_min;
  const double a11 = cyy - lambda_min;
  const double a22 = czz - lambda_min;

  // Берем кросс-произведения строк матрицы для нахождения нормали к её ядру
  geometry::Vector3D row0{a00, cxy, cxz};
  geometry::Vector3D row1{cxy, a11, cyz};
  geometry::Vector3D row2{cxz, cyz, a22};

  geometry::Vector3D eigenvec = row0.cross(row1);
  if (eigenvec.length() < math::eps) { eigenvec = row0.cross(row2); }
  if (eigenvec.length() < math::eps) { eigenvec = row1.cross(row2); }
  if (eigenvec.length() < math::eps) { return best_plane; }

  return geometry::Plane(centroid, eigenvec);
}

}  // namespace lidar
