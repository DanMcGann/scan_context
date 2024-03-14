/** @brief This file implements the ScanContext descriptor interface defined in scan_context.h
 *
 *  @author Dan McGann
 *  @date March 2024
 */
#pragma once
#include <cmath>
#include <numeric>

#include "scan_context/scan_context.h"

/**
 * ########     ###    ########     ###    ##     ##  ######
 * ##     ##   ## ##   ##     ##   ## ##   ###   ### ##    ##
 * ##     ##  ##   ##  ##     ##  ##   ##  #### #### ##
 * ########  ##     ## ########  ##     ## ## ### ##  ######
 * ##        ######### ##   ##   ######### ##     ##       ##
 * ##        ##     ## ##    ##  ##     ## ##     ## ##    ##
 * ##        ##     ## ##     ## ##     ## ##     ##  ######
 */

/*********************************************************************************************************************/
template <class PointType>
bool ScanContext<PointType>::Params::equals(const Params& other) const {
  return this->number_sectors == other.number_sectors &&  //
         this->number_rings == other.number_rings &&      //
         this->max_range == other.max_range;
}

/*********************************************************************************************************************/
template <class PointType>
bool ScanContext<PointType>::Params::operator()(const Params& other) const {
  return this.equals(other);
}

/**
 *  ######   ######     ###    ##    ##  ######   #######  ##    ## ######## ######## ##     ## ########
 * ##    ## ##    ##   ## ##   ###   ## ##    ## ##     ## ###   ##    ##    ##        ##   ##     ##
 * ##       ##        ##   ##  ####  ## ##       ##     ## ####  ##    ##    ##         ## ##      ##
 *  ######  ##       ##     ## ## ## ## ##       ##     ## ## ## ##    ##    ######      ###       ##
 *       ## ##       ######### ##  #### ##       ##     ## ##  ####    ##    ##         ## ##      ##
 * ##    ## ##    ## ##     ## ##   ### ##    ## ##     ## ##   ###    ##    ##        ##   ##     ##
 *  ######   ######  ##     ## ##    ##  ######   #######  ##    ##    ##    ######## ##     ##    ##
 */

/*********************************************************************************************************************/
template <class PointType>
ScanContext<PointType>::ScanContext(const std::vector<PointType>& lidar_scan, const Params& params) : params_(params) {
  // Allocate the descriptor matrix and ring_key vector
  descriptor_ = Eigen::MatrixXd::Zero(params_.number_rings, params.number_sectors);
  ring_key_ = Eigen::VectorXd::Zero(params_.number_rings);

  // Compute the resolutions used for computing ring and sector indicies
  const double ring_resolution = params_.max_range / params_.number_rings;  // meters
  const double sector_resolution = (2 * M_PI) / params_.number_sectors;     // radians

  // Track the minimum height in the scan so we can normalize the descriptor to be positive starting at zero
  // WARN: This is only ever implied in [1] in Fig. 1, never explicitly outlined
  double min_height = std::numeric_limits<double>::max();

  // Iterate over all points
  for (const PointType& point : lidar_scan) {
    // Compute range to point
    const double range = std::sqrt(point(0) * point(0) + point(1) * point(1));                // meters
    const double angle_rad = std::fmod(std::atan2(point(1), point(0)) + 2 * M_PI, 2 * M_PI);  // radians
    const double height = point(2);                                                           // meters
    // Include points only they are within the parameterized range
    if (range < params_.max_range) {
      // Get the index for this point
      const size_t ring_idx = static_cast<size_t>(range / ring_resolution);
      const size_t sector_idx = static_cast<size_t>(angle_rad / sector_resolution);

      // Update the descriptor at the corresponding location
      if (descriptor_(ring_idx, sector_idx) == 0.0) {  // If uninitialized, initialize the cell
        descriptor_(ring_idx, sector_idx) = height;
      } else {  // If initialized, take the max
        descriptor_(ring_idx, sector_idx) = std::max(descriptor_(ring_idx, sector_idx), height);
      }

      // Update the min height for later normalization
      min_height = std::min(min_height, height);
    }

    // TODO construct the ring_Key
  }

  // Normalize the descriptor according to the min height, leaving uninitialized cells as zero
  for (size_t r = 0; r < params_.number_rings; r++) {
    for (size_t s = 0; s < params.number_sectors; s++) {
      if (descriptor_(r, s) != 0.0) descriptor_(r, s) -= min_height;
    }
  }
}

/*********************************************************************************************************************/
template <class PointType>
const Eigen::MatrixXd& ScanContext<PointType>::descriptor() const {
  return descriptor_;
}

/*********************************************************************************************************************/
template <class PointType>
const Eigen::VectorXd& ScanContext<PointType>::ringKey() const {
  return ring_key_;
}

/*********************************************************************************************************************/
template <class PointType>
double ScanContext<PointType>::distance(const ScanContext<PointType>& other) const {
  // Guard Code: Two ScanContexts are only comparable if they are constructed with identical parameters
  if (!params_.equals(other.params_)) {
    throw std::runtime_error(
        "ScanContext::distance The two ScanContexts are not comparable due to different parameters.");
  }

  // Compute the min distance over all column shifts ("rotations") of this descriptor
  double min_distance = std::numeric_limits<double>::max();
  for (size_t sector_offset = 0; sector_offset < params_.number_sectors; sector_offset++) {
    min_distance = std::min(min_distance, shiftedDistance(sector_offset, other));
  }
  return min_distance;
}

/*********************************************************************************************************************/
template <class PointType>
double ScanContext<PointType>::ringKeyDistance(const ScanContext<PointType>& other) const {
  return (ring_key_ - other.ring_key_).norm();
}

/*********************************************************************************************************************/
template <class PointType>
double ScanContext<PointType>::shiftedDistance(const size_t& sector_offset, const ScanContext<PointType>& other) const {
  double sum_term = 0.0;
  for (size_t sector_idx = 0; sector_idx < params_.number_sectors; sector_idx++) {
    // Compute the sector for this sector idx after the rotation by sector_offset
    size_t this_offset_sector_idx = (sector_idx + sector_offset) % params_.number_sectors;
    // Extract the respective columns from the two descriptors
    const Eigen::VectorXd this_sector = descriptor_.col(this_offset_sector_idx);
    const double this_norm = this_sector.norm();

    const Eigen::VectorXd other_sector = other.descriptor_.col(sector_idx);
    const double other_norm = other_sector.norm();

    /** Compute the cosine distance between the two sectors
     * Note: there are two edge cases, if either sector is empty
     * If both are empty we say the distance is zero
     * If only one is empty we say distance is equal two 1 (i.e. vectors are orthogonal)
     * WARN: These edge cases are never addressed in [1]
     */
    if (this_norm > 0 && other_norm > 0) {  // Standard Cosine Distance
      sum_term += 1 - ((this_sector.dot(other_sector)) / (this_sector.norm() * other_sector.norm()));
    } else if (this_norm == 0 && other_norm == 0) {  // All zero sectors, assume zero distance
      sum_term += 0;
    } else {  // One zero and one non-zero sector, assume orthogonal distance
      sum_term += 1;
    }
  }

  // Average the summed cosine distances
  return sum_term / params_.number_sectors;
}