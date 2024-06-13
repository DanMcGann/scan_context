#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>
#include <random>

#include "scan_context/database.h"
#include "scan_context/scan_context.h"

/**
 * ##     ## ######## ##       ########  ######## ########   ######
 * ##     ## ##       ##       ##     ## ##       ##     ## ##    ##
 * ##     ## ##       ##       ##     ## ##       ##     ## ##
 * ######### ######   ##       ########  ######   ########   ######
 * ##     ## ##       ##       ##        ##       ##   ##         ##
 * ##     ## ##       ##       ##        ##       ##    ##  ##    ##
 * ##     ## ######## ######## ##        ######## ##     ##  ######
 */

std::vector<Eigen::Vector3d> randomPCD(size_t num_pts, double min = -60, double max = 60) {
  std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
  std::uniform_real_distribution<double> dist(min, max);
  std::vector<Eigen::Vector3d> pcd;
  for (size_t i = 0; i < num_pts; i++) {
    pcd.push_back(Eigen::Vector3d(dist(generator), dist(generator), dist(generator)));
  }
  return pcd;
}

ScanContext randomSC(size_t num_pts, ScanContext::Params params) {
  // Will generate pts outside of max range, due to uniform sampling, this is desired
  std::vector<Eigen::Vector3d> pcd = randomPCD(num_pts, -params.max_range, params.max_range);
  return ScanContext::fromScan<ParenAccessor>(pcd, params);
}

/**
 *  ######   #######  ##    ##  ######  ########    ###    ##    ## ########  ######
 * ##    ## ##     ## ###   ## ##    ##    ##      ## ##   ###   ##    ##    ##    ##
 * ##       ##     ## ####  ## ##          ##     ##   ##  ####  ##    ##    ##
 * ##       ##     ## ## ## ##  ######     ##    ##     ## ## ## ##    ##     ######
 * ##       ##     ## ##  ####       ##    ##    ######### ##  ####    ##          ##
 * ##    ## ##     ## ##   ### ##    ##    ##    ##     ## ##   ###    ##    ##    ##
 *  ######   #######  ##    ##  ######     ##    ##     ## ##    ##    ##     ######
 */

std::pair<ScanContext::Params, ScanContextDatabase::Params> params() {
  ScanContext::Params sc_params;
  sc_params.max_range = 40;
  sc_params.number_sectors = 60;
  sc_params.number_rings = 20;

  ScanContextDatabase::Params db_params;
  db_params.sc_params = sc_params;
  db_params.number_ring_key_nn = 5;
  db_params.kdtree_rebuild_threshold = 100;

  return std::make_pair(sc_params, db_params);
}

/**
 * ######## ########  ######  ########  ######
 *    ##    ##       ##    ##    ##    ##    ##
 *    ##    ##       ##          ##    ##
 *    ##    ######    ######     ##     ######
 *    ##    ##             ##    ##          ##
 *    ##    ##       ##    ##    ##    ##    ##
 *    ##    ########  ######     ##     ######
 */
/*********************************************************************************************************************/
TEST(TestScanContextDatabase, Correctness) {
  auto [sc_params, db_params] = params();
  db_params.number_ring_key_nn = 1;

  ScanContextDatabase database(db_params);
  std::vector<ScanContext> all_sc;

  // Add descriptors to the database
  for (size_t i = 0; i < 125; i++) {
    auto sc = randomSC(5000, sc_params);
    database.insert(i, sc);
    all_sc.push_back(sc);
  }

  // Delete some of the descriptors(do not go over rebuild thresh)
  for (size_t i = 0; i < 25; i++) {
    database.remove(i);
  }

  // Generate a query
  ScanContext query = randomSC(5000, sc_params);
  std::optional<size_t> idx = database.query(query);
  ASSERT_TRUE(idx);

  // Ensure that this is in fact the closest according to ring key
  ScanContext closest = all_sc[*idx];
  double closest_distance = query.ringKeyDistance(closest);
  for (size_t i = 25; i < 125; i++) {
    ASSERT_LE(closest_distance, query.ringKeyDistance(all_sc[i]));
  }
}

/*********************************************************************************************************************/
TEST(TestScanContextDatabase, EmptyTree) {
  auto [sc_params, db_params] = params();
  // Generate the database
  ScanContextDatabase database(db_params);
  // Generate a query
  ScanContext query = randomSC(5000, sc_params);

  // Add descriptors to the database
  for (size_t i = 0; i < 125; i++) {
    auto sc = randomSC(500, sc_params);
    database.insert(i, sc);
  }

  // Delete Entire databse
  for (size_t i = 0; i < 125; i++) {
    database.remove(i);
  }

  // test
  std::optional<size_t> idx = database.query(query);
  ASSERT_FALSE(idx);
}

/*********************************************************************************************************************/
TEST(TestScanContextDatabase, DeletedTree) {
  auto [sc_params, db_params] = params();
  // Generate the database
  ScanContextDatabase database(db_params);
  // Generate a query
  ScanContext query = randomSC(5000, sc_params);
  // test
  std::optional<size_t> idx = database.query(query);
  ASSERT_FALSE(idx);
}

/*********************************************************************************************************************/
TEST(TestScanContextDatabase, KnnGreaterThanTreeSize) {
  auto [sc_params, db_params] = params();
  // Generate the database
  ScanContextDatabase database(db_params);
  database.insert(0, randomSC(5000, sc_params));
  // Generate a query
  ScanContext query = randomSC(5000, sc_params);
  // test
  std::optional<size_t> idx = database.query(query);
  ASSERT_TRUE(idx);
  ASSERT_EQ(*idx, 0);
}

/*********************************************************************************************************************/
TEST(TestScanContextDatabase, KGreaterThanTreeSize) {
  auto [sc_params, db_params] = params();
  // Generate the database
  ScanContextDatabase database(db_params);
  database.insert(0, randomSC(5000, sc_params));
  // Generate a query
  ScanContext query = randomSC(5000, sc_params);
  // test
  std::vector<size_t> result = database.queryK(query, 10);
  ASSERT_EQ(result.size(), 1);
  ASSERT_EQ(result[0], 0);
}

/*********************************************************************************************************************/
TEST(TestScanContextDatabase, TestThreshold) {
  auto [sc_params, db_params] = params();
  // Generate the database
  ScanContextDatabase database(db_params);
  database.insert(0, randomSC(5000, sc_params));
  database.insert(1, randomSC(5000, sc_params));
  // Generate a query
  // Distance will be ~ 1 to an empty descriptor
  ScanContext query = ScanContext(sc_params);
  // test
  std::optional<size_t> idx = database.query(query);
  ASSERT_FALSE(idx);
}

/********************************************************************************************************************
TEST(TestScanContextDatabase, ProfileRebuild100) {
  auto [sc_params, db_params] = params();

  ScanContextDatabase database(db_params);

  // Add descriptors to the database
  double insert_total = 0;
  double query_total = 0;
  for (size_t i = 0; i < 1000; i++) {
    auto sc = randomSC(1000, sc_params);
    auto q = randomSC(1000, sc_params);

    // Insert
    auto insert_time_start = std::chrono::high_resolution_clock::now();
    database.insert(i, sc);
    auto insert_time_end = std::chrono::high_resolution_clock::now();
    insert_total +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(insert_time_end - insert_time_start).count() * 1e-9;

    // Query
    auto query_time_start = std::chrono::high_resolution_clock::now();
    database.query(q);
    auto query_time_end = std::chrono::high_resolution_clock::now();
    query_total +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(query_time_end - query_time_start).count() * 1e-9;
  }

  std::cout << "Avg Insert: " << insert_total / 1000.0 << std::endl;
  std::cout << "Avg Query: " << query_total / 1000.0 << std::endl;
}
*/

/*******************************************************************************************************************
TEST(TestScanContextDatabase, ProfileRebuild1) {
  auto [sc_params, db_params] = params();
  db_params.kdtree_rebuild_threshold = 1;

  ScanContextDatabase database(db_params);

  // Add descriptors to the database
  double insert_total = 0;
  double query_total = 0;
  for (size_t i = 0; i < 1000; i++) {
    auto sc = randomSC(1000, sc_params);
    auto q = randomSC(1000, sc_params);

    // Insert
    auto insert_time_start = std::chrono::high_resolution_clock::now();
    database.insert(i, sc);
    auto insert_time_end = std::chrono::high_resolution_clock::now();
    insert_total +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(insert_time_end - insert_time_start).count() * 1e-9;

    // Query
    auto query_time_start = std::chrono::high_resolution_clock::now();
    database.query(q);
    auto query_time_end = std::chrono::high_resolution_clock::now();
    query_total +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(query_time_end - query_time_start).count() * 1e-9;
  }

  std::cout << "Avg Insert: " << insert_total / 1000.0 << std::endl;
  std::cout << "Avg Query: " << query_total / 1000.0 << std::endl;
}
**/