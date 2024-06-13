/** @brief Class definition for the Scan Context Database.
 * This database is used to store ScanContext descriptors, and query matches using the 2 stage matching process outlined
 * in [1].
 *
 * [1] G. Kim and A. Kim, "Scan Context: Egocentric Spatial Descriptor for Place Recognition Within 3D Point Cloud Map,"
 * 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, 2018, pp. 4802-4809,
 *
 * Design: Currently there are no good options for incremental maintenance of KDtrees used for nearest neighbor search
 * of ScanContext ring keys. As such we will be forced to entirely re-build the KDtree periodically. We parameterize
 * when to perform this re-build by the number of modifications (additions / removals) from the database. Note that
 * from the external interface, this internal behavior is hidden by queuing new additions and removals. Queued new
 * additions are linearly searched over when finding nearest neighbors, and queued removals are automatically removed
 * from any returned result.
 *
 * @author Dan McGann
 * @date March 2024
 */
#pragma once
#include <scan_context/scan_context.h>

#include <map>
#include <memory>
#include <nanoflann.hpp>
#include <optional>
#include <set>

class ScanContextDatabase {
  /** TYPES **/
 public:
  struct Params {
    //// @brief The parameters for all ScanContexts added to the database.
    //// All ScanContexts added to the database must use the same params to ensure they are comparable
    typename ScanContext::Params sc_params;

    /// @brief The number of ring key nearest neighbors over which to compare ScanContext distances
    size_t number_ring_key_nn{5};

    /// @brief The max cosine distance between matched ScanContexts
    /// All nearest neighbors with with a distance greater than this thresh are not considered when querying the
    /// database
    double query_distance_threshold{0.5};

    /// @brief The number of modifications to the KDtree required to initiate a re-build.
    //// Where a modification is either a addition or a removal
    size_t kdtree_rebuild_threshold{100};
  };

 protected:
  /// @brief Adaptor class from a vector of ScanContexts to a nanoflann::KDTree compatible dataset
  //// [required by nanoflann]
  struct ScanContextKDTreeAdaptor {
    std::vector<Eigen::VectorXd> data;
    // Interface required by nanoflann
    size_t kdtree_get_point_count() const { return data.size(); }
    double kdtree_get_pt(const size_t idx, const size_t dim) const { return data.at(idx)(dim); }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const {
      return false;
    }
    ScanContextKDTreeAdaptor(const std::vector<Eigen::VectorXd> data = {}) : data(data) {}
  };

  /// @brief Type for the KDTree distance metric [required by nanoflann]
  typedef nanoflann::L2_Simple_Adaptor<double, ScanContextKDTreeAdaptor> KDTreeDistance;
  /// @brief Type for the KDTree using the ring size configured at runtime
  typedef nanoflann::KDTreeSingleIndexAdaptor<KDTreeDistance, ScanContextKDTreeAdaptor> KDTree;
  typedef nanoflann::KDTreeSingleIndexAdaptorParams KDTreeParams;

  /** Fields **/
 protected:
  /// @brief The params for the Database
  Params params_;
  /// @brief The internal KDTree used for nearest neighbor search
  std::shared_ptr<KDTree> kd_tree_;
  ScanContextKDTreeAdaptor tree_ring_keys_;
  /// @brief index_[i] stores the key of the ScanContext associated with the ring key at node i in the KDTree
  std::vector<size_t> index_;

  /// @brief The internal database of ScanContext descriptors
  std::map<size_t, ScanContext> database_;

  /// @brief The queue of ScanContexts to add to the database during the next rebuild
  std::map<size_t, ScanContext> insertion_queue_;
  /// @brief The queue of ScanContexts to remove from the database during the next rebuild
  std::set<size_t> removal_queue_;

  /** Interface **/
 public:
  /** @brief Constructor for an empty database
   * @param params: The parameters that configure the database behavior
   */
  ScanContextDatabase(const Params& params) : params_(params) {
    kd_tree_ = std::make_shared<KDTree>(params_.sc_params.number_rings, tree_ring_keys_, KDTreeParams(20));
  }

  /** @brief Adds the given ScanContext to the Database
   * @param Key: The unique identifier for scan_context being added
   * @param scan_context: The descriptor to add to the database
   * @throws std::invalid_argument - If key already exists in the database
   * @throws std::invalid_argument - If the scan_context uses different params than the database
   * WARN: This function can be slow if params.kdtree_rebuild_threshold is hit and we initiate a rebuild of the KDTree
   */
  void insert(size_t key, const ScanContext& scan_context);

  /** @brief Removes the ScanContext associated with the given key from the database
   * @param key: The unique identifier for the scan_context to be removed
   * WARN: nullopt if the key is not in the database
   */
  void remove(size_t key);

  /** @brief Queries the database returning the key of the ScanContext that most matches the query
   * @param query: The query ScanContext for which we are finding the closest match in the database
   * @returns The closest matching element in the database or null if none exist
   */
  std::optional<size_t> query(const ScanContext& query) const;

  /** @brief Queries the database for the keys of the top k ScanContexts that most match the query
   * @param query: The query ScanContext for which we are finding the closest match in the database
   * @param k: The max number of matched ScanContexts to return
   * @returns The top k ScanContexts that match the query
   * WARN: k may override Params::number_ring_key_nn when searching for nearest ring keys
   * WARN: Less than k results may be returned if there are not k valid results
   */
  std::vector<size_t> queryK(const ScanContext& query, size_t k) const;

  /** HELPERS **/
 protected:
  /** @brief Rebuilds the internal index and KDTree.This is a potentially expensive operation as we have to, from
   * scratch, reconstruct the internal KDTree of RingKeys. A rebuild will only occur when called if the threshold
   * modifications has been exceeded (@see Params::kdtree_rebuild_threshold)
   */
  void tryRebuild();

  /** @brief Retrieves the k nearest neighbors from the ring key tree accounting for the removal_queue_
   * @param query: The query ScanContext for which we are finding the closest match in the database
   * @param k: The max number of matched ScanContexts to return
   * @returns The top k ScanContexts that match the query if there are sufficient points in the database
   * WARN: May query the underlying kdtree multiple times
   */
  std::vector<std::pair<size_t, double>> findRingKeyNeighborsSafe(const ScanContext& query, size_t k) const;

  /** @brief We commonly store references to descriptors with their distances pair[key, distance]
   * This function provides comparison to use in standard algorithms on such paris
   *
   */
  static bool keyDistPairComp(const std::pair<size_t, double>& lhs, const std::pair<size_t, double>& rhs) {
    return lhs.second < rhs.second;
  }
};
