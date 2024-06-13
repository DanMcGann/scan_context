/** @brief Class definition for the Scan Context descriptor.
 * ScanContext is a lidar scan descriptor introduced by [1] used for place recognition.
 * The following library implements ScanContext exactly as described in [1] using only the paper as reference.
 *
 * [1] G. Kim and A. Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition Within 3D Point Cloud Map,"
 * 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, 2018, pp. 4802-4809,
 *
 * @author Dan McGann
 * @date March 2024
 */

#pragma once
#include <scan_context/types.h>

#include <Eigen/Dense>
#include <cstddef>
#include <vector>

/// @brief The Scan Context Descriptor
class ScanContext {
  /** TYPES **/
 public:
  /// @brief The parameters for defining the descriptor resolution.
  /// @note In order to compare two descriptors they much have matching parameters
  struct Params {
    /// @brief The number of azimuth sectors in which to segment the lidar scan.
    /// Azimuth Bin resolution is therefore 2π/num_sectors.
    size_t number_sectors{60};
    /// @brief The number of radial rings in which to segment the lidar scan.
    /// Radial Bin resolution is therefore max_range/number_rings.
    size_t number_rings{20};
    /// @brief The maximum range of the lidar sensor (meters).
    /// All points beyond this range are ignored, and this helps to define radial bin resolution
    double max_range{80};

    bool equals(const Params& other) const;
  };

  /** Fields **/
 protected:
  /// @brief The parameters used to construct this ScanContext descriptor
  Params params_;
  /// @brief The ScanContext descriptor for a lidar scan
  /// Of size [number_rings, number_sectors] where each row corresponds to a ring of lidar data
  Eigen::MatrixXd descriptor_;
  /// @brief The ring key descriptor, a rotational invariant summary of the ScanContext descriptor
  /// Of size [number_rings] where each element is the L1 norm of a row in descriptor_
  Eigen::VectorXd ring_key_;

  /** Interface **/
 public:
  /// @brief Constructs an Empty ScanContext. See fromScan to construct from a lidar Scan.
  ScanContext(const Params& params);

  /** @brief Constructs a ScanContext descriptor from a lidar scan
   * @tparam Accessor - Class used to access the x, y, z data from points in the point clouds
   * @tparam PointType - The type of the point in the pointcloud must contain x, y, z data in some form
   * @tparam Alloc - The allocator used for the pointcloud vector
   * @param lidar_scan: The lidar scan to describe
   * @param params: The ScanContext parameters
   * @return A ScanContext descriptor for lidar_scan
  */
  template <template <typename> class Accessor = FieldAccessor, typename PointType, template <typename> class Alloc>
  static ScanContext fromScan(const std::vector<PointType, Alloc<PointType>>& lidar_scan, const Params& params);

  /// @brief Accessor to the descriptor that guards against modification
  const Eigen::MatrixXd& descriptor() const;

  /// @brief Accessor to the ring key that guards against modification
  const Eigen::VectorXd& ringKey() const;

  /// @brief Accessor to the params that guards against modification
  const Params& params() const;

  /** @brief Computes the distance between this descriptor and the other descriptor
   * Distance is defined as as the column-wise cosine distance minimized over all possible column shifts.
   * See [1] Eq.6 for exact details.
   * @param other: The other ScanContext descriptor with which to compare distances
   * @returns The distance between the two descriptors
   * @note Distance is symmetric i.e. this.distance(other) == other.distance(this)
   * @note we can only compute distances for "comparable" descriptors i.e. those with equal params
   */
  double distance(const ScanContext& other) const;

  /** @brief Computes the distance between the ring key of this descriptor and the ring key of the other descriptor.
   * Distance is defined as the euclidean distance between the two ring key vectors.
   * @param other: The other ScanContext descriptor with which to compare ring key distances
   * @returns The ring key distance between two descriptors
   * @note By definition ring key distance is symmetric
   */
  double ringKeyDistance(const ScanContext& other) const;

  /** Helpers **/
 private:
  /** @brief Computes the distance between this ScanContext "rotated" by the given sector offset and the other.
   * Used as a helper in ScanContext::distance, and is ebullient to [1] Eq.5
   * @param sector_offset: The sector offset used to "rotate" this descriptor when comparing to other
   * @param other: The other descriptor with which we are comparing
   * @returns The cosine distance between this rotated by sector offset and other
   * @note Assumes that this and other are comparable
   */
  double shiftedDistance(const size_t& sector_offset, const ScanContext& other) const;
};

/// Include the implementation of the descriptor
#include "scan_context/scan_context-inl.h"