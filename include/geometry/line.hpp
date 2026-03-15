#pragma once

#include <optional>

#include "vector_3d.hpp"

namespace geometry {

// -- line -- (r = origin + t * dir)
class Line {
 public:
  Vector3D origin;
  Vector3D dir;

  Line(const Vector3D& origin, const Vector3D& dir);

  [[nodiscard]] bool is_valid() const noexcept;
  [[nodiscard]] bool is_match(const Line& other) const noexcept;
  [[nodiscard]] bool is_parallel(const Line& other) const noexcept;
  [[nodiscard]] bool is_intersect(const Line& other) const noexcept;
  [[nodiscard]] bool is_contains(const Vector3D& point) const noexcept;

  [[nodiscard]] std::optional<Vector3D> intersect_point(
      const Line& other) const noexcept;
  void print() const;
};
}  // namespace geometry