/** @brief The purpose of this file is to define some types and helper functions that are used throughout the
 * implementation to support generic handling of points
 * @author Dan McGann
 * @date June 2024
 */
#pragma once

/// @brief Accessor struct for points that store their position as public fields
// Assumes that fields are named x, y, and z, example: PCL Point Type
template <typename PointType>
struct FieldAccessor {
  static double x(PointType pt) { return pt.x; }
  static double y(PointType pt) { return pt.y; }
  static double z(PointType pt) { return pt.z; }
};

/// @brief Accessor struct for points that store their positions in a vector/array/matrix accessed with brackets
/// Assumes that order is [x, y, z] and zero indexed, example: Eigen Matrix
template <typename PointType>
struct ParenAccessor {
  static double x(PointType pt) { return pt(0); }
  static double y(PointType pt) { return pt(1); }
  static double z(PointType pt) { return pt(2); }
};

/// @brief Accessor struct for points that store their positions in a vector/array/matrix accessed with an `at` func
/// Assumes that order is [x, y, z] and zero indexed, example: std::vector
template <typename PointType>
struct AtAccessor {
  static double x(PointType pt) { return pt.at(0); }
  static double y(PointType pt) { return pt.at(1); }
  static double z(PointType pt) { return pt.at(2); }
};