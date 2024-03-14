#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "scan_context/scan_context.h"

typedef ScanContext<Eigen::Vector3d> ScanContextEigen;

TEST(TestScanContext, SimpleConstruction) {
  ScanContextEigen::Params params;
  params.max_range = 10;
  params.number_rings = 1;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 0));
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 2));

  ScanContextEigen sc(lidar_scan, params);
  ASSERT_NEAR(sc.descriptor()(0, 0), 1.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 1), 2.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 2), 0.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 3), 0.0, 1e-9);
}

TEST(TestScanContext, ConstructionWithNegativeHeight) {
  ScanContextEigen::Params params;
  params.max_range = 10;
  params.number_rings = 1;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 2));
  lidar_scan.push_back(Eigen::Vector3d(-1, -1, -2));
  lidar_scan.push_back(Eigen::Vector3d(-1, -1, -1));

  ScanContextEigen sc(lidar_scan, params);
  ASSERT_NEAR(sc.descriptor()(0, 0), 3.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 1), 4.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 2), 1.0, 1e-9);
  ASSERT_NEAR(sc.descriptor()(0, 3), 0.0, 1e-9);
}

TEST(TestScanContext, EquivDistance) {
  ScanContextEigen::Params params;
  params.max_range = 10;
  params.number_rings = 1;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 2));

  ScanContextEigen sc(lidar_scan, params);
  ScanContextEigen sc2(lidar_scan, params);
  ASSERT_NEAR(sc.distance(sc2), 0.0, 1e-9);
}

TEST(TestScanContext, EquivDistanceRotated) {
  ScanContextEigen::Params params;
  params.max_range = 10;
  params.number_rings = 1;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 2));

  ScanContextEigen sc(lidar_scan, params);

  std::vector<Eigen::Vector3d> lidar_scan_2;
  lidar_scan_2.push_back(Eigen::Vector3d(-1, -1, 1));
  lidar_scan_2.push_back(Eigen::Vector3d(-1, -1, 1));
  lidar_scan_2.push_back(Eigen::Vector3d(-1, -1, 2));

  ScanContextEigen sc2(lidar_scan_2, params);
  ASSERT_NEAR(sc.distance(sc2), 0.0, 1e-9);
}

TEST(TestScanContext, DiffDistance) {
  ScanContextEigen::Params params;
  params.max_range = 10;
  params.number_rings = 2;
  params.number_sectors = 4;

  std::vector<Eigen::Vector3d> lidar_scan;
  lidar_scan.push_back(Eigen::Vector3d(1, 1, 3));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan.push_back(Eigen::Vector3d(-1, 1, 2));

  ScanContextEigen sc(lidar_scan, params);

  std::vector<Eigen::Vector3d> lidar_scan_2;
  lidar_scan_2.push_back(Eigen::Vector3d(1, 1, 3));
  lidar_scan_2.push_back(Eigen::Vector3d(1, 8, 3));
  lidar_scan_2.push_back(Eigen::Vector3d(-1, 1, 1));
  lidar_scan_2.push_back(Eigen::Vector3d(-1, 1, 2));
  lidar_scan_2.push_back(Eigen::Vector3d(-1, -1, 2));
  ScanContextEigen sc2(lidar_scan_2, params);

  ASSERT_GT(sc.distance(sc2), 0.0);
}
