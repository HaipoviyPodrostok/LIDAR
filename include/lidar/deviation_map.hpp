#pragma once

#include <string>

#include "geometry/plane.hpp"
#include "lidar/lidar_processor.hpp"

namespace lidar {

[[nodiscard]] std::string render_deviation_map(const DepthView& depth_map,
                                               const RegionMask& mask,
                                               const LidarConfig& config,
                                               const geometry::Plane& plane,
                                               double threshold_mm,
                                               const std::string& output_name);

}  // namespace lidar
