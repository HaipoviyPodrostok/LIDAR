#pragma once

#include <cmath>

namespace math {

inline constexpr double eps     = 1e-6;
inline constexpr double abs_tol = 1e-6;
inline constexpr double rel_tol = 1e-6;

inline constexpr double inner_area_width = 100.0;

enum class Axis {
    X = 0,
    Y = 1,
    Z = 2,
};

enum class CmpResult {
    Less    = -1,
    Equal   =  0,
    Greater =  1,
};

template <typename T>
inline CmpResult cmp(T a, T b,
                     T rel = static_cast<T>(rel_tol),
                     T abs = static_cast<T>(abs_tol))
{
    if (std::isnan(a) || std::isnan(b)) {
        return CmpResult::Greater;
    }

    if (std::isinf(a) || std::isinf(b)) {
        if (a == b) return CmpResult::Equal;
        return (a < b) ? CmpResult::Less : CmpResult::Greater;
    }

    T diff  = std::fabs(a - b);
    T scale = std::max(std::fabs(a), std::fabs(b));

    if (diff <= abs) {
        return CmpResult::Equal;
    }

    if (diff <= rel * scale) {
        return CmpResult::Equal;
    }

    return (a < b) ? CmpResult::Less : CmpResult::Greater;
}

template <typename T>
inline bool is_equal(T a, T b,
                     T rel = static_cast<T>(rel_tol),
                     T abs = static_cast<T>(abs_tol)) {
    
    return cmp(a, b, rel, abs) == CmpResult::Equal;
}

inline bool is_zero(double x,
                    double scale = 1.0,
                    double rel = rel_tol,
                    double abs = abs_tol) {
    
    double eff_abs = abs;
    double eff_rel = rel * scale;
    double diff = std::fabs(x);

    if (diff <= eff_abs) { return true; }
    return diff <= eff_rel;
}

template <typename T>
inline T get_eps(T scale = 1.0, 
                 T abs = static_cast<T>(abs_tol),
                 T rel = static_cast<T>(rel_tol)) {
    return std::max(abs, rel * scale);
}

template <typename T>
constexpr T sqr(T x) { return x * x; }

} // namespace math