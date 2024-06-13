#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "scan_context/scan_context.h"
#include "scan_context/types.h"

TEST(TestScanContext, SimpleConstruction) {
  ScanContext::Params params;
  params.max_range = 10;
  params.number_rings = 1;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 0));
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 2));

  ScanContext sc = ScanContext::fromScan<ParenAccessor>(lidar_scan, params);
  ASSERT_NEAR(sc.descriptor()(0, 0), 1.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 1), 2.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 2), 0.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 3), 0.0, 1e-9);
}

/*********************************************************************************************************************/
TEST(TestScanContext, ConstructionWithNegativeHeight) {
  ScanContext::Params params;
  params.max_range = 10;
  params.number_rings = 1;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 2));
  lidar_scan.push_back(Eigen::Vector3d(-1, -1, -2));
  lidar_scan.push_back(Eigen::Vector3d(-1, -1, -1));

  ScanContext sc = ScanContext::fromScan<ParenAccessor>(lidar_scan, params);
  ASSERT_NEAR(sc.descriptor()(0, 0), 3.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 1), 4.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 2), 1.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 3), 0.0, 1e-9);
}

/*********************************************************************************************************************/
TEST(TestScanContext, EquivDistance) {
  ScanContext::Params params;
  params.max_range = 10;
  params.number_rings = 1;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 2));

  ScanContext sc = ScanContext::fromScan<ParenAccessor>(lidar_scan, params);
  ScanContext sc2 = ScanContext::fromScan<ParenAccessor>(lidar_scan, params);
  ASSERT_NEAR(sc.distance(sc2), 0.0, 1e-9);
}

/*********************************************************************************************************************/
TEST(TestScanContext, EquivDistanceRotated) {
  ScanContext::Params params;
  params.max_range = 10;
  params.number_rings = 1;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 2));

  ScanContext sc = ScanContext::fromScan<ParenAccessor>(lidar_scan, params);

  std::vector<Eigen::Vector3d> lidar_scan_2;
  lidar_scan_2.push_back(Eigen::Vector3d(-1, -1, 1));
  lidar_scan_2.push_back(Eigen::Vector3d(-1, -1, 1));
  lidar_scan_2.push_back(Eigen::Vector3d(-1, -1, 2));

  ScanContext sc2 = ScanContext::fromScan<ParenAccessor>(lidar_scan_2, params);
  ASSERT_NEAR(sc.distance(sc2), 0.0, 1e-9);
}

/*********************************************************************************************************************/
TEST(TestScanContext, DiffDistance) {
  ScanContext::Params params;
  params.max_range = 10;
  params.number_rings = 2;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 3));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 2));

  ScanContext sc = ScanContext::fromScan<ParenAccessor>(lidar_scan, params);

  std::vector<Eigen::Vector3d> lidar_scan_2;
  lidar_scan_2.push_back(Eigen::Vector3d(1, 1, 3));
  lidar_scan_2.push_back(Eigen::Vector3d(1, 8, 3));
  lidar_scan_2.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan_2.push_back(Eigen::Vector3d(-1, 1, 2));
  lidar_scan_2.push_back(Eigen::Vector3d(-1, -1, 2));
  ScanContext sc2 = ScanContext::fromScan<ParenAccessor>(lidar_scan_2, params);

  ASSERT_GT(sc.distance(sc2), 0.0);
}

/*********************************************************************************************************************/
TEST(TestScanContext, TestRingKeySimple) {
  ScanContext::Params params;
  params.max_range = 10;
  params.number_rings = 2;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 0));
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 5));

  ScanContext sc = ScanContext::fromScan<ParenAccessor>(lidar_scan, params);
  ASSERT_NEAR(sc.ringKey()(0), 1.0 / 4.0, 1e-9);
  ASSERT_NEAR(sc.ringKey()(1), 0, 1e-9);
}

/*********************************************************************************************************************/
TEST(TestScanContext, TestRingKeyComplex) {
  ScanContext::Params params;
  params.max_range = 10;
  params.number_rings = 2;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 0));
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 5));

  lidar_scan.push_back(Eigen::Vector3d(5, 5, 5));
  lidar_scan.push_back(Eigen::Vector3d(-5, -5, 5));
  lidar_scan.push_back(Eigen::Vector3d(5, -5, 5));

  ScanContext sc = ScanContext::fromScan<ParenAccessor>(lidar_scan, params);
  ASSERT_NEAR(sc.ringKey()(0), 1.0 / 4.0, 1e-9);
  ASSERT_NEAR(sc.ringKey()(1), 3.0 / 4.0, 1e-9);
}

/*********************************************************************************************************************/
TEST(TestScanContext, CustomAllocator) {
  ScanContext::Params params;
  params.max_range = 10;
  params.number_rings = 1;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 0));
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 2));

  ScanContext sc = ScanContext::fromScan<ParenAccessor>(lidar_scan, params);
  ASSERT_NEAR(sc.descriptor()(0, 0), 1.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 1), 2.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 2), 0.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 3), 0.0, 1e-9);
}