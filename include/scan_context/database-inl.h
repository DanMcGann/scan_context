/** @brief This file implements the ScanContext Database interface defined in database.h
 *
 *  @author Dan McGann
 *  @date March 2024
 */
#pragma once

#include "scan_context/database.h"

/*********************************************************************************************************************/
template <class PointType>
void ScanContextDatabase<PointType>::insert(size_t key, const ScanContext<PointType>& scan_context) {
  // Guard Code
  if (!params_.sc_params.equals(scan_context.params())) {
    throw std::invalid_argument(
        "ScanContextDatabase::insert provided scan_context with mis-matched parameters to database");
  }
  if (database_.count(key) != 0 || insertion_queue_.count(key) != 0) {
    throw std::invalid_argument("ScanContextDatabase::insert provided key already exists in the database");
  }
  // Add the new descriptor to the insertion queue
  insertion_queue_.insert(std::make_pair(key, scan_context));
  // Rebuild the database if necessary
  tryRebuild();
}

/*********************************************************************************************************************/
template <class PointType>
void ScanContextDatabase<PointType>::remove(size_t key) {
  // Try to remove from the insertion queue, if nothing is removed, add it to the queue
  if (!insertion_queue_.erase(key)) {
    removal_queue_.insert(key);
  }
  // Rebuild the database if necessary
  tryRebuild();
}

/*********************************************************************************************************************/
template <class PointType>
std::optional<size_t> ScanContextDatabase<PointType>::query(const ScanContext<PointType>& query) const {
  std::vector<size_t> matches = queryK(query, 1);
  if (matches.size()) {
    return matches[0];
  } else {
    return std::nullopt;
  }
}

/*********************************************************************************************************************/
template <class PointType>
std::vector<size_t> ScanContextDatabase<PointType>::queryK(const ScanContext<PointType>& query, size_t k) const {
  // Determine the number of ring-key nearest neighbors to retrieve
  size_t number_rink_key_nn = std::max(k, params_.number_ring_key_nn);

  // First find the K nearest rink key neighbors accounting for the removal queue
  std::vector<std::pair<size_t, double>> ring_key_neighbors = findRingKeyNeighborsSafe(query, number_rink_key_nn);

  // Find distance to the farthest nearest neighbor
  double worst_ring_key_distance = std::numeric_limits<double>::max();
  if (ring_key_neighbors.size() > 0) {
    worst_ring_key_distance =
        (std::max_element(ring_key_neighbors.begin(), ring_key_neighbors.end(), keyDistPairComp))->second;
  }

  // Traverse the insertion queue and add any element closer than the worst tree neighbor
  for (const auto& kvp : insertion_queue_) {
    double dist = query.ringKeyDistance(kvp.second);
    if (dist < worst_ring_key_distance) ring_key_neighbors.push_back(std::make_pair(kvp.first, dist));
  }

  // Sort all the ring key neighbors according to their distance
  std::sort(ring_key_neighbors.begin(), ring_key_neighbors.end(), keyDistPairComp);

  // For the top number_rink_key_nn compute their ScanContext Distance
  std::vector<std::pair<size_t, double>> scan_context_neighbors;
  for (size_t i = 0; i < std::min(number_rink_key_nn, ring_key_neighbors.size()); i++) {
    const size_t key = ring_key_neighbors[i].first;
    const ScanContext<PointType> candidate = database_.count(key) ? database_.at(key) : insertion_queue_.at(key);
    const double distance = query.distance(candidate);
    if (distance < params_.query_distance_threshold) {
      scan_context_neighbors.push_back(std::make_pair(key, distance));
    }
  }

  // Sort all the scan context neighbors according to their distance
  std::sort(scan_context_neighbors.begin(), scan_context_neighbors.end(), keyDistPairComp);

  // Accumulate the final results
  std::vector<size_t> knn;
  for (size_t i = 0; i < std::min(k, scan_context_neighbors.size()); i++) {
    knn.push_back(scan_context_neighbors[i].first);
  }
  return knn;
}

/*********************************************************************************************************************/
template <class PointType>
void ScanContextDatabase<PointType>::tryRebuild() {
  // Do not rebuild until we have sufficient modifications [early exit]
  if ((insertion_queue_.size() + removal_queue_.size()) < params_.kdtree_rebuild_threshold) return;

  // Insert all queued descriptors to the database
  database_.insert(insertion_queue_.begin(), insertion_queue_.end());
  insertion_queue_.clear();

  // Remove all queued descriptors from the database
  for (const size_t& k : removal_queue_) database_.erase(k);
  removal_queue_.clear();

  // Construct the index for the KDTree and organize the ringKeys
  tree_ring_keys_ = ScanContextKDTreeAdaptor();
  index_ = std::vector<size_t>();
  for (auto& kvp : database_) {
    index_.push_back(kvp.first);
    tree_ring_keys_.data.push_back(kvp.second.ringKey());
  }
  kd_tree_ = std::make_shared<KDTree>(params_.sc_params.number_rings, tree_ring_keys_, KDTreeParams(20));
}

/*********************************************************************************************************************/
template <class PointType>
std::vector<std::pair<size_t, double>> ScanContextDatabase<PointType>::findRingKeyNeighborsSafe(
    const ScanContext<PointType>& query, size_t k) const {
  bool continue_search = true;
  std::vector<std::pair<size_t, double>> neighbors;
  size_t num_results = k;
  while (continue_search) {
    // Setup the KNN search
    std::vector<size_t> knn_indicies(num_results);
    std::vector<double> knn_distances_sq(num_results);
    nanoflann::KNNResultSet<double> result_set(num_results);
    result_set.init(&knn_indicies[0], &knn_distances_sq[0]);
    // Do the KNN search
    kd_tree_->findNeighbors(result_set, &query.ringKey()[0]);

    // Compose the results accounting for removed keys
    for (size_t i = 0; i < result_set.size(); i++) {
      size_t key = index_[knn_indicies[i]];
      if (removal_queue_.count(key) == 0) {
        neighbors.push_back(std::make_pair(key, std::sqrt(knn_distances_sq[i])));
      }
    }

    // We are complete only if we have exhausted the all points in the database or we have found the knn
    if (neighbors.size() == k || result_set.size() < num_results || result_set.size() >= database_.size()) {
      continue_search = false;
    } else {
      neighbors.clear();
      num_results += k;
    }
  }

  return neighbors;
}
