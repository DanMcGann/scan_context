/** @brief This file partially implements the ScanContext descriptor interface defined in scan_context.h
 *
 *  @author Dan McGann
 *  @date March 2024
 */
#pragma once
#include <cmath>
#include <iostream>
#include <numeric>

#include "scan_context/scan_context.h"

/*********************************************************************************************************************/
template <template <typename> class Accessor = FieldAccessor, typename PointType, template <typename> class Alloc>
ScanContext ScanContext::fromScan(const std::vector<PointType, Alloc<PointType>>& lidar_scan, const Params& params) {
  ScanContext sc(params);

  // Compute the resolutions used for computing ring and sector indicies
  const double ring_resolution = sc.params_.max_range / sc.params_.number_rings;  // meters
  const double sector_resolution = (2 * M_PI) / sc.params_.number_sectors;     // radians

  // Track the minimum height in the scan so we can normalize the descriptor to be positive starting at zero
  // WARN: This is only ever implied in [1] in Fig. 1, never explicitly outlined
  double min_height = std::numeric_limits<double>::max();

  // Iterate over all points
  for (const PointType& point : lidar_scan) {
    // Compute range to point
    const double x = Accessor<PointType>::x(point);                             // meters
    const double y = Accessor<PointType>::y(point);                             // meters
    const double height = Accessor<PointType>::z(point);                        // meters
    const double range = std::sqrt(x * x + y * y);                              // meters
    const double angle_rad = std::fmod(std::atan2(y, x) + 2 * M_PI, 2 * M_PI);  // radians

    // Include points only they are within the parameterized range
    if (range < sc.params_.max_range) {
      // Get the index for this point
      const size_t ring_idx = static_cast<size_t>(range / ring_resolution);
      const size_t sector_idx = static_cast<size_t>(angle_rad / sector_resolution);

      // Update the descriptor at the corresponding location
      if (sc.descriptor_(ring_idx, sector_idx) == 0.0) {  // If uninitialized, initialize the cell
        sc.descriptor_(ring_idx, sector_idx) = height;
      } else {  // If initialized, take the max
        sc.descriptor_(ring_idx, sector_idx) = std::max(sc.descriptor_(ring_idx, sector_idx), height);
      }

      // Update the min height for later normalization
      min_height = std::min(min_height, height);
    }
  }

  // Normalize the descriptor according to the min height, leaving uninitialized cells as zero
  for (size_t r = 0; r < sc.params_.number_rings; r++) {
    for (size_t s = 0; s < sc.params_.number_sectors; s++) {
      if (sc.descriptor_(r, s) != 0.0) sc.descriptor_(r, s) -= min_height;
    }
  }

  // Construct the ring key for the now complete ScanContext
  for (size_t r = 0; r < sc.params_.number_rings; r++) {
    sc.ring_key_(r) = static_cast<double>(sc.descriptor_.row(r).array().count()) / sc.params_.number_sectors;
  }

  return sc;
}
