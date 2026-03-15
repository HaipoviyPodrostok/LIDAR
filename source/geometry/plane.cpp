#include "geometry/plane.hpp"

#include <cassert>
#include <optional>

#include "geometry/line.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math.hpp"

namespace geometry {

Plane::Plane(const Vector3D& point, const Vector3D& n) noexcept : r(point) {
  assert(point.is_valid() && n.is_valid() && !n.is_zero());
  normal = n / n.length();
  D = normal.scalar(point);
}

bool Plane::is_valid() const noexcept {
  return (r.is_valid() && normal.is_valid() && !normal.is_zero());
}

double Plane::get_distance(const Plane& other) const noexcept {
  assert(this->is_valid());
  assert(other.is_valid());
  double sign = (normal.is_codirected(other.normal)) ? 1.0 : -1.0;
  return std::fabs(D - sign * other.D);
}

bool Plane::is_match(const Plane& other) const noexcept {
  assert(this->is_valid());
  assert(other.is_valid());

  if (!normal.is_collinear(other.normal)) { return false; }

  return math::is_zero(get_distance(other));
}

bool Plane::is_parallel(const Plane& other) const noexcept {
  assert(this->is_valid());
  assert(other.is_valid());

  if (!normal.is_collinear(other.normal)) { return false; }

  return !math::is_zero(get_distance(other));
}

bool Plane::is_contains(const Vector3D& p) const noexcept {
  assert(this->is_valid());
  assert(p.is_valid());
  return math::is_zero(p.scalar(normal) - D);
}

bool Plane::is_contains(const Line& line) const noexcept {
  assert(this->is_valid());
  assert(line.is_valid());

  return math::is_zero(normal.scalar(line.dir), line.dir.length()) &&
         this->is_contains(line.origin);
}

bool Plane::is_intersected(const Line& line) const noexcept {
  assert(this->is_valid());
  assert(line.is_valid());

  return !math::is_zero(normal.scalar(line.dir)) || this->is_contains(line);
}

std::optional<Vector3D> Plane::get_intersect_point(
    const Line& line) const noexcept {
  assert(this->is_valid());
  assert(line.is_valid());
  if (!is_intersected(line)) { return std::nullopt; }

  double denom = normal.scalar(line.dir);
  double num = normal.scalar(line.origin) + D;

  assert(!(math::is_zero(denom) && !math::is_zero(num)));
  if (math::is_zero(denom)) { return line.origin; }

  double t = -num / denom;
  return line.origin + line.dir * t;
}
}  // namespace geometry