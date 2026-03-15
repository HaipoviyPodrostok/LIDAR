#pragma once

#include <math.h>

#include <cassert>

#include "line.hpp"
#include "plane.hpp"
#include "section.hpp"
#include "vector_3d.hpp"

namespace geometry {

class Triangle {
 public:
  Vector3D a;
  Vector3D b;
  Vector3D c;

  Triangle(const Vector3D& a, const Vector3D& b, const Vector3D& c) noexcept;

  [[nodiscard]] Plane get_plane() const noexcept;
  [[nodiscard]] Line get_intersect_line(const Triangle& other) const noexcept;
  void print() const noexcept;
  [[nodiscard]] Vector3D get_centre() const noexcept;

  [[nodiscard]] bool is_valid() const noexcept;
  [[nodiscard]] bool is_intersect(const Section& sec) const noexcept;
  [[nodiscard]] bool is_intersect(const Triangle& other) const noexcept;
  [[nodiscard]] bool is_point() const noexcept;
  [[nodiscard]] bool is_section() const noexcept;
  [[nodiscard]] bool is_inside(const Vector3D& p) const noexcept;

 private:
  [[nodiscard]] bool is_intersect_2d(const Triangle& other) const noexcept;
  [[nodiscard]] bool is_intersect_3d(const Triangle& other) const noexcept;
};
}  // namespace geometry