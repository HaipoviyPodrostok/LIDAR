#include "geometry/line.hpp"

#include <cassert>
#include <iostream>

#include "geometry/vector_3d.hpp"
#include "math/math.hpp"

namespace geometry {

Line::Line(const Vector3D& origin, const Vector3D& dir)
    : origin(origin), dir(dir) {
  assert(origin.is_valid());
  assert(dir.is_valid());
  assert(!dir.is_zero());
}

bool Line::is_valid() const noexcept {
  return origin.is_valid() && dir.is_valid() && !(dir.is_zero());
}

bool Line::is_match(const Line& other) const noexcept {
  assert(this->is_valid());
  assert(other.is_valid());
  return dir.is_collinear(other.dir) &&
         (origin - other.origin).is_collinear(dir);
}

bool Line::is_parallel(const Line& other) const noexcept {
  assert(this->is_valid());
  assert(other.is_valid());
  return dir.is_collinear(other.dir) &&
         !(origin - other.origin).is_collinear(dir);
}

bool Line::is_contains(const Vector3D& point) const noexcept {
  assert(this->is_valid());
  assert(point.is_valid());
  return ((point - origin).is_collinear(dir));
}

bool Line::is_intersect(const Line& other) const noexcept {
  assert(is_valid());
  assert(other.is_valid());

  if (is_match(other)) { return true; }
  if (is_parallel(other)) { return false; }

  const Vector3D v = origin - other.origin;
  const Vector3D n = dir.cross(other.dir);
  const double scale = v.length() * n.length();

  return math::is_zero(v.scalar(n), scale);
}

// -- line this:  p1 + t * d1 --
// -- line other: p2 + s * d2 --
std::optional<Vector3D> Line::intersect_point(
    const Line& other) const noexcept {
  assert(is_valid());
  assert((other.is_valid()));
  if (!this->is_intersect(other) || this->is_match(other)) {
    return std::nullopt;
  }

  const Vector3D& p1 = origin;
  const Vector3D& d1 = dir;
  const Vector3D& p2 = other.origin;
  const Vector3D& d2 = other.dir;

  const Vector3D n = d1.cross(d2);
  assert(!n.is_zero());
  const double n_len_squared = n.length() * n.length();

  const double t = ((p2 - p1).cross(d2)).scalar(n) / n_len_squared;
  const double s = ((p2 - p1).cross(d1)).scalar(n) / n_len_squared;

  if (!(std::isfinite(t) && std::isfinite(s))) { return std::nullopt; }

  const Vector3D a = p1 + d1 * t;
  const Vector3D b = p2 + d2 * s;

  double scale = std::max(std::max(a.length(), b.length()), 1.0);
  if (math::is_zero((a - b).length(), scale)) { return (a + b) * 0.5; }

  return std::nullopt;
}

void Line::print() const {
  std::cout << "origin = ";
  origin.print();
  std::cout << ", dir = ";
  dir.print();
}
}  // namespace geometry