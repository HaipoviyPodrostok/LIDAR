#include "geometry/vector_3d.hpp"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>

#include "math/math.hpp"

namespace geometry {

bool Vector3D::is_valid() const {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

Vector3D operator+(const Vector3D& lhs, const Vector3D& rhs) noexcept {
  assert(lhs.is_valid() && rhs.is_valid());
  return Vector3D{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vector3D operator-(const Vector3D& lhs, const Vector3D& rhs) noexcept {
  assert(lhs.is_valid() && rhs.is_valid());
  return Vector3D{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vector3D operator*(const Vector3D& v, double scalar) noexcept {
  assert(v.is_valid());
  return Vector3D{v.x * scalar, v.y * scalar, v.z * scalar};
}

Vector3D operator*(double scalar, const Vector3D& v) noexcept {
  assert(v.is_valid());
  return Vector3D{v.x * scalar, v.y * scalar, v.z * scalar};
}

Vector3D operator/(const Vector3D& v, double scalar) noexcept {
  assert(v.is_valid() && !math::is_zero(scalar));
  return v * (1 / scalar);
}

Vector3D operator/(double scalar, const Vector3D& v) noexcept {
  assert(v.is_valid() && !math::is_zero(scalar));
  return v * (1 / scalar);
}

double& Vector3D::operator[](size_t idx) {
  assert(this->is_valid());
  assert(idx < 3);
  if (idx == 0) { return x; }
  if (idx == 1) { return y; }
  return z;
}

const double& Vector3D::operator[](size_t idx) const {
  assert(this->is_valid());
  assert(idx < 3);
  if (idx == 0) { return x; }
  if (idx == 1) { return y; }
  return z;
}

bool Vector3D::is_collinear(const Vector3D& other) const {
  assert(this->is_valid());
  assert(other.is_valid());

  const double scale = this->length() * other.length();
  return ((this->cross(other)).is_zero(scale));
}

bool Vector3D::is_codirected(const Vector3D& other) const {
  assert(this->is_valid());
  assert(other.is_valid());
  return (is_collinear(other) && this->scalar(other) >= 0.0f);
}

bool Vector3D::is_match(const Vector3D& other) const {
  assert(this->is_valid());
  assert(other.is_valid());
  return ((*this - other).is_zero());
}

bool Vector3D::is_zero(double scale) const {
  assert(this->is_valid());
  return (math::is_zero(length(), scale));
}

double Vector3D::length() const {
  assert(this->is_valid());
  return (std::sqrt(x * x + y * y + z * z));
}

double Vector3D::scalar(const Vector3D& other) const {
  assert(this->is_valid());
  assert(other.is_valid());
  return (x * other.x + y * other.y + z * other.z);
}

Vector3D Vector3D::cross(const Vector3D& other) const {
  assert(this->is_valid());
  assert(other.is_valid());

  double i = (y * other.z) - (z * other.y);
  double j = (z * other.x) - (x * other.z);
  double k = (x * other.y) - (y * other.x);

  return Vector3D{i, j, k};
}

void Vector3D::print() const {
  std::cout << "(" << x << ", " << y << ", " << z << ")";
}
}  // namespace geometry