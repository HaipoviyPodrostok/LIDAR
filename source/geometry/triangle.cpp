#include "geometry/triangle.hpp"

#include <endian.h>

#include <cassert>
#include <cstddef>
#include <iostream>
#include <vector>

#include "geometry/plane.hpp"
#include "geometry/section.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math.hpp"

namespace geometry {

Triangle::Triangle(const Vector3D& a, const Vector3D& b,
                   const Vector3D& c) noexcept
    : a(a), b(b), c(c) {
  assert(a.is_valid() && b.is_valid() && c.is_valid());
};

bool Triangle::is_valid() const noexcept {
  return a.is_valid() && b.is_valid() && c.is_valid();
}

void Triangle::print() const noexcept {
  std::cout << "A = ";
  a.print();
  std::cout << ", B = ";
  b.print();
  std::cout << ", C = ";
  c.print();
}

bool Triangle::is_point() const noexcept {
  assert(this->is_valid());
  return (b - a).is_zero() && (c - a).is_zero();
}

bool Triangle::is_section() const noexcept {
  assert(this->is_valid());
  if (is_point()) { return false; }

  const Vector3D ab = b - a;
  const Vector3D bc = c - b;
  const Vector3D ac = c - a;

  const double scale_ab = ab.length() * ab.length();
  const double scale_bc = bc.length() * bc.length();
  const double scale_ac = ac.length() * ac.length();

  if (ab.is_zero(scale_ab) || bc.is_zero(scale_bc) || ac.is_zero(scale_ac)) {
    return true;
  }

  if ((ab.cross(ac)).is_zero(ab.length() * ac.length())) { return true; }
  return false;
}

bool Triangle::is_intersect(const Section& sec) const noexcept {
  assert(this->is_valid());
  assert(!is_point() && !is_section());
  assert(sec.is_valid());

  Plane pl = this->get_plane();
  Line l = sec.get_line();

  if (!pl.is_intersected(l)) { return false; }
  if (pl.is_contains(l)) {
    return Section{a, b}.is_intersect(sec) || Section{b, c}.is_intersect(sec) ||
           Section{a, c}.is_intersect(sec) || is_inside(sec.a) ||
           is_inside(sec.b);
  }
  auto opt_x = pl.get_intersect_point(l);
  assert(opt_x.has_value());
  const Vector3D& x = *opt_x;
  return sec.is_contains(x) && this->is_inside(x);
}

bool Triangle::is_intersect(const Triangle& other) const noexcept {
  assert(this->is_valid());
  assert(other.is_valid());

  if (this->is_point() && other.is_point()) { return (a - other.a).is_zero(); }

  auto pick_section = [](const Vector3D& a, const Vector3D& b,
                         const Vector3D& c) {
    Vector3D p = a;
    Vector3D q = b;

    if ((p - q).is_zero()) { q = c; }

    return std::pair<Vector3D, Vector3D>{p, q};
  };

  if (this->is_point() && other.is_section()) {
    auto [p, q] = pick_section(other.a, other.b, other.c);
    return Section{p, q}.is_contains(a);
  }

  if (this->is_section() && other.is_point()) {
    auto [p, q] = pick_section(a, b, c);
    return Section{p, q}.is_contains(other.a);
  }

  if (this->is_section() && other.is_section()) {
    auto [p, q] = pick_section(a, b, c);
    auto [m, n] = pick_section(other.a, other.b, other.c);
    return Section{p, q}.is_intersect(Section{m, n});
  }

  if (this->is_point()) { return other.is_inside(a); }

  if (other.is_point()) { return this->is_inside(other.a); }

  if (this->is_section()) {
    auto [p, q] = pick_section(a, b, c);
    return other.is_intersect(Section{p, q});
  }

  if (other.is_section()) {
    auto [p, q] = pick_section(other.a, other.b, other.c);
    return this->is_intersect(Section{p, q});
  }

  const Plane first_pl = this->get_plane();
  const Plane second_pl = other.get_plane();

  if (first_pl.is_match(second_pl)) { return is_intersect_2d(other); }

  if (first_pl.is_parallel(second_pl)) { return false; }

  return is_intersect_3d(other);
}

Plane Triangle::get_plane() const noexcept {
  assert(this->is_valid());
  assert(!this->is_point());
  assert(!this->is_section());

  const Vector3D ab = b - a;
  const Vector3D ac = c - a;

  const Vector3D normal = ab.cross(ac);

  return Plane{a, normal};
}

bool Triangle::is_inside(const Vector3D& p) const noexcept {
  assert(this->is_valid());
  assert(p.is_valid());
  if (!this->get_plane().is_contains(p)) { return false; }

  const Vector3D ab = b - a;
  const Vector3D bc = c - b;
  const Vector3D ca = a - c;
  const Vector3D ap = p - a;
  const Vector3D bp = p - b;
  const Vector3D cp = p - c;

  const Vector3D normal = ab.cross(bc);

  const double side_ab = (normal.cross(ab)).scalar(ap);
  const double side_bc = (normal.cross(bc)).scalar(bp);
  const double side_ca = (normal.cross(ca)).scalar(cp);

  double scale_ab = (normal.cross(ab)).length() * ap.length();
  double scale_bc = (normal.cross(bc)).length() * bp.length();
  double scale_ca = (normal.cross(ca)).length() * cp.length();

  double eps_ab = math::eps * scale_ab;
  double eps_bc = math::eps * scale_bc;
  double eps_ca = math::eps * scale_ca;

  return side_ab >= -eps_ab && side_bc >= -eps_bc && side_ca >= -eps_ca;
}

bool Triangle::is_intersect_2d(const Triangle& other) const noexcept {
  assert(this->is_valid());
  assert(other.is_valid());
  assert(this->get_plane().is_match(other.get_plane()));

  const Section ab{a, b};
  const Section bc{b, c};
  const Section ca{c, a};
  const Section other_ab{other.a, other.b};
  const Section other_bc{other.b, other.c};
  const Section other_ca{other.c, other.a};

  const Section* sides1[3] = {&ab, &bc, &ca};
  const Section* sides2[3] = {&other_ab, &other_bc, &other_ca};

  if (is_inside(other.a) || is_inside(other.b) || is_inside(other.c)) {
    return true;
  }

  if (other.is_inside(a) || other.is_inside(b) || other.is_inside(c)) {
    return true;
  }

  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      if (sides1[i]->is_intersect(*sides2[j])) { return true; };
    }
  }

  return false;
}

bool Triangle::is_intersect_3d(const Triangle& other) const noexcept {
  assert(this->is_valid());
  assert(other.is_valid());

  const Section ab{a, b};
  const Section bc{b, c};
  const Section ca{c, a};
  const Section other_ab{other.a, other.b};
  const Section other_bc{other.b, other.c};
  const Section other_ca{other.c, other.a};

  const Line intersect_line = get_intersect_line(other);
  assert(intersect_line.is_valid());

  const Section* sides1[3] = {&ab, &bc, &ca};
  std::vector<Vector3D> intersect_points1;

  for (size_t i = 0; i < 3; i++) {
    if (sides1[i]->is_belong(intersect_line)) {
      intersect_points1.push_back(sides1[i]->a);
      intersect_points1.push_back(sides1[i]->b);
    } else if (sides1[i]->is_intersect(intersect_line)) {
      auto opt_p = sides1[i]->intersect_point(intersect_line);
      assert(opt_p.has_value());
      intersect_points1.push_back(*opt_p);
    }
  }

  if (intersect_points1.size() < 2) { return false; }

  const Section* sides2[3] = {&other_ab, &other_bc, &other_ca};
  std::vector<Vector3D> intersect_points2;

  for (size_t i = 0; i < 3; i++) {
    if (sides2[i]->is_belong(intersect_line)) {
      intersect_points2.push_back(sides2[i]->a);
      intersect_points2.push_back(sides2[i]->b);
    } else if (sides2[i]->is_intersect(intersect_line)) {
      auto opt_p = sides2[i]->intersect_point(intersect_line);
      assert(opt_p.has_value());
      intersect_points2.push_back(*opt_p);
    }
  }

  if (intersect_points2.size() < 2) { return false; }

  if (intersect_points1[0].is_match(intersect_points1[1])) {
    Vector3D p1 = intersect_points1[0];

    if (intersect_points2[0].is_match(intersect_points2[1])) {
      Vector3D p2 = intersect_points2[0];
      return p1.is_match(p2);
    }

    Section s2(intersect_points2[0], intersect_points2[1]);
    return s2.is_contains(p1);
  }

  if (intersect_points2[0].is_match(intersect_points2[1])) {
    Vector3D p2 = intersect_points2[0];
    Section s1(intersect_points1[0], intersect_points1[1]);
    return s1.is_contains(p2);
  }

  const Section s1{intersect_points1[0], intersect_points1[1]};
  const Section s2{intersect_points2[0], intersect_points2[1]};

  return s1.is_intersect(s2);
}

Line Triangle::get_intersect_line(const Triangle& other) const noexcept {
  assert(this->is_valid());
  assert(other.is_valid());

  const Plane pl1 = get_plane();
  const Plane pl2 = other.get_plane();

  assert(!pl1.is_parallel(pl2));
  assert(!pl1.is_match(pl2));

  const Vector3D n1 = pl1.get_normal();
  const Vector3D n2 = pl2.get_normal();

  const Vector3D dir = n1.cross(n2);

  const Vector3D pl_intersect_p =
      (n2 * pl1.get_D() - n1 * pl2.get_D()).cross(n1.cross(n2)) /
      math::sqr((n1.cross(n2)).length());

  return {pl_intersect_p, dir};
}

Vector3D Triangle::get_centre() const noexcept {
  assert(this->is_valid());

  return {(a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0,
          (a.z + b.z + c.z) / 3.0};
}

}  // namespace geometry