#pragma once

#include "line.hpp"
#include "vector_3d.hpp"

namespace geometry {

// -- plane -- ((r, n) = D)
class Plane {
 public:
  Plane(const Vector3D& point, const Vector3D& n) noexcept;

  [[nodiscard]] const Vector3D& get_r() const noexcept { return r; }
  [[nodiscard]] const Vector3D& get_normal() const noexcept { return normal; }
  [[nodiscard]] double get_D() const noexcept { return D; }

  [[nodiscard]] bool is_valid() const noexcept;
  [[nodiscard]] double get_distance(const Plane& other) const noexcept;
  [[nodiscard]] bool is_match(const Plane& other) const noexcept;
  [[nodiscard]] bool is_parallel(const Plane& other) const noexcept;
  [[nodiscard]] bool is_contains(const Vector3D& p) const noexcept;
  [[nodiscard]] bool is_contains(const Line& line) const noexcept;
  [[nodiscard]] bool is_intersected(const Line& line) const noexcept;
  [[nodiscard]] std::optional<Vector3D> get_intersect_point(
      const Line& line) const noexcept;

 private:
  Vector3D r;
  Vector3D normal;
  double D;
};
}  // namespace geometry