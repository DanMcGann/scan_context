/** @brief This file partially implements the ScanContext descriptor interface defined in scan_context.h
 *
 *  @author Dan McGann
 *  @date March 2024
 */
#include <cmath>
#include <iostream>
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
bool ScanContext::Params::equals(const Params& other) const {
  // Check all parameters are equal
  return this->number_sectors == other.number_sectors &&  //
         this->number_rings == other.number_rings &&      //
         this->max_range == other.max_range;
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
ScanContext::ScanContext(const Params& params) : params_(params) {
  descriptor_ = Eigen::MatrixXd::Zero(params_.number_rings, params.number_sectors);
  ring_key_ = Eigen::VectorXd::Zero(params_.number_rings);
}

/*********************************************************************************************************************/
const Eigen::MatrixXd& ScanContext::descriptor() const { return descriptor_; }

/*********************************************************************************************************************/
const Eigen::VectorXd& ScanContext::ringKey() const { return ring_key_; }

/*********************************************************************************************************************/
const typename ScanContext::Params& ScanContext::params() const { return params_; }

/*********************************************************************************************************************/
double ScanContext::distance(const ScanContext& other) const {
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
double ScanContext::ringKeyDistance(const ScanContext& other) const { return (ring_key_ - other.ring_key_).norm(); }

/*********************************************************************************************************************/
double ScanContext::shiftedDistance(const size_t& sector_offset, const ScanContext& other) const {
  double sum_term = 0.0;
  for (size_t sector_idx = 0; sector_idx < params_.number_sectors; sector_idx++) {
    // Compute the sector for this sector idx after the rotation by sector_offset
    const size_t this_offset_sector_idx = (sector_idx + sector_offset) % params_.number_sectors;
    // Extract the respective columns from the two descriptors
    const Eigen::VectorXd this_sector = descriptor_.col(this_offset_sector_idx);
    const double this_norm = this_sector.norm();
    const Eigen::VectorXd other_sector = other.descriptor_.col(sector_idx);
    const double other_norm = other_sector.norm();

    /** Compute the cosine distance between the two sectors
     * Note: there are two edge cases, if either sector is empty
     * If both are empty we say the distance is zero
     * If only one is empty we say distance is equal two 1 (i.e. vectors are orthogonal)
     * WARN: These edge cases are never addressed in [1], so these are based on intuition
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