#pragma once

#include <optional>

#include "line.hpp"
#include "vector_3d.hpp"

namespace geometry {

class Section {
 public:
  Vector3D a;
  Vector3D b;

  Section(const Vector3D& a, const Vector3D& b);

  [[nodiscard]] bool is_valid() const noexcept;
  [[nodiscard]] bool is_intersect(const Section& other) const noexcept;
  [[nodiscard]] bool is_intersect(const Line& other) const noexcept;
  [[nodiscard]] bool is_belong(const Line& line) const noexcept;
  [[nodiscard]] std::optional<Vector3D> intersect_point(
      const Line& other) const noexcept;
  [[nodiscard]] double length() const noexcept;
  [[nodiscard]] Line get_line() const noexcept;
  [[nodiscard]] bool is_contains(const Vector3D& p) const noexcept;

  void print() const;
};
}  // namespace geometry