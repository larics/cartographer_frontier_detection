// clang-format off
#pragma GCC target ("arch=broadwell")
// clang-format on

//#pragma GCC push_options
//#pragma GCC optimize ("O0")

#include <absl/synchronization/mutex.h>
#include <cartographer/mapping/2d/map_limits.h>
#include <cartographer_ros/frontier_detection.h>
#include <cartographer_ros/msg_conversion.h>

int total_submap_updates = 0;
int optimization_events = 0;
int skipped_updates = 0;

namespace frontier {

Detector::Detector(cartographer::mapping::PoseGraph* const pose_graph)
    : publisher_initialized_(false),
      last_optimizations_performed_(-1),
      pose_graph_(pose_graph),
      submaps_(pose_graph_) {
  lambda_worker_.start();
  optimization_timer_ = ros::NodeHandle().createWallTimer(
      ::ros::WallDuration(0.2), &Detector::CheckOptimizationEventsPeriodicallyWhenIdle, this);
}

void Detector::NotifyEnd() {
  optimization_timer_.stop();
  lambda_worker_.finish();
}

void Detector::InitPublisher() {
  frontier_publisher_ =
      ros::NodeHandle().advertise<visualization_msgs::MarkerArray>(
          "frontier_marker", 3, true);
  publisher_initialized_ = true;
}

void Detector::PublishAllSubmaps() {
  if (!publisher_initialized_) InitPublisher();
  visualization_msgs::MarkerArray frontier_markers;

  std::unique_lock<std::mutex> lock(mutex_);

  for (const auto& submap_i : submap_frontier_points_) {
    frontier_markers.markers.push_back(
        CreateMarkerForSubmap(submap_i.first, nullptr /* updated_submap_ids */,
                              true /* check_against_active */));
  }

  frontier_publisher_.publish(frontier_markers);
}

void Detector::PublishSubmaps(
    const std::vector<cartographer::mapping::SubmapId>& submap_ids,
    const std::vector<cartographer::mapping::SubmapId>& additional_submaps) {
  if (!publisher_initialized_) InitPublisher();

  visualization_msgs::MarkerArray frontier_markers;

  for (const auto& id_i : submap_ids) {
    // LOG(ERROR) << "publishing submap " << id_i.submap_index;
    frontier_markers.markers.push_back(
        CreateMarkerForSubmap(id_i, nullptr /* updated_submap_ids */,
                              false /* check_against_active */));
  }

  for (const auto& id_additional : additional_submaps) {
    // LOG(ERROR) << "publishing submap " << id_i.submap_index;
    frontier_markers.markers.push_back(CreateMarkerForSubmap(
        id_additional, &submap_ids /* updated_submap_ids */,
        false /* check_against_active */));
  }

  frontier_publisher_.publish(frontier_markers);
}

bool Detector::CheckForOptimizationEvent() {
  int actual_optimizations_performed = pose_graph_->optimizations_performed();
  if (actual_optimizations_performed != last_optimizations_performed_) {
    last_optimizations_performed_ = actual_optimizations_performed;
  } else
    return false;
  optimization_events++;
  submaps_.Invalidate();
  RebuildTree();
  PublishAllSubmaps();
  return true;
}

void Detector::CheckOptimizationEventsPeriodicallyWhenIdle(
    const ::ros::WallTimerEvent&) {
  if (lambda_worker_.TaskCount() == 0) {
    lambda_worker_.PushIntoWorkQueue([this]() { CheckForOptimizationEvent(); });
  }
}

visualization_msgs::Marker& Detector::CreateMarkerForSubmap(
    const cartographer::mapping::SubmapId& id_i,
    const std::vector<cartographer::mapping::SubmapId>* const
        updated_submap_ids,
    const bool check_against_active) {
  Submap& s_i(submaps_(id_i));

  s_i.frontier_marker.points.clear();

  std::vector<double> updated_frontier_marker_points_global;

  if (updated_submap_ids == nullptr) {
    // When recomputing all submaps, or when computing only updated submaps.
    auto& submap_frontier_points = submap_frontier_points_.at(s_i.id);
    auto& bounding_box = bounding_boxes_.at(s_i.id);

    std::vector<Value> intersecting_submaps;
    intersecting_submaps.reserve(32);
    rt_.query(bgi::intersects(bounding_box.last_global_box),
              std::back_inserter(intersecting_submaps));

    updated_frontier_marker_points_global.reserve(
        static_cast<size_t>(submap_frontier_points.first.cols()) * 2);

    Eigen::Matrix3Xd submap_frontier_points_global =
        (s_i.to_global_position * submap_frontier_points.first);
    for (int i = 0; i < submap_frontier_points_global.cols(); i++) {
      const auto global_position = submap_frontier_points_global.col(i);

      bool ok = true;
      auto& submap_hint = submap_frontier_points.second.at(i);

      if (submap_hint != cartographer::mapping::SubmapId{-1, -1}) {
        Submap* s_j = submaps_.IfExists(submap_hint);
        if (s_j == nullptr) {
          submap_hint = {-1, -1};
        } else {
          if (s_j->is_known(s_j->to_local_submap_position * global_position))
            ok = false;
        }
        if (ok) submap_hint = {-1, -1};
      }

      if (ok && check_against_active) {
        for (const auto& active_submap : active_submaps_) {
          const cartographer::mapping::SubmapId& id_j = active_submap;
          if (id_j == s_i.id ||
              (id_j.trajectory_id == s_i.id.trajectory_id &&
               (abs(id_j.submap_index - s_i.id.submap_index) <= 2)))
            continue;
          if (!bg::intersects(
                  bounding_box.last_global_box,
                  bounding_boxes_.at(active_submap).last_global_box)) {
            continue;
          }

          Submap* s_j = submaps_.IfExists(id_j);
          if (s_j == nullptr) {
            continue;
          }
          if (s_j->is_known(s_j->to_local_submap_position * global_position)) {
            ok = false;
            submap_hint = s_j->id;
            break;
          }
        }
      }

      if (ok) {
        for (const auto& intersecting_submap : intersecting_submaps) {
          const cartographer::mapping::SubmapId& id_j =
              intersecting_submap.second;
          if (id_j == s_i.id ||
              (id_j.trajectory_id == s_i.id.trajectory_id &&
               (abs(id_j.submap_index - s_i.id.submap_index) <= 2))) {
            continue;
          }

          Submap* s_j = submaps_.IfExists(id_j);
          if (s_j == nullptr) {
            rt_.remove(
                std::make_pair(bounding_boxes_.at(id_j).last_global_box, id_j));
            continue;
          }

          if (s_j->is_known(s_j->to_local_submap_position * global_position)) {
            ok = false;
            submap_hint = id_j;
            break;
          }
        }
      }

      if (ok) {
        updated_frontier_marker_points_global.insert(
            updated_frontier_marker_points_global.end(),
            {global_position.x(), global_position.y()});
        geometry_msgs::Point frontier_point;
        frontier_point.x = global_position.x();
        frontier_point.y = global_position.y();
        frontier_point.z = 0.;  // global_position.z();
        s_i.frontier_marker.points.push_back(frontier_point);
        // submap_hint = {-1, -1};
      }
    }
  } else {
    // Computing the intersecting submaps. Check if frontier points were
    // covered by an active submap update.
    updated_frontier_marker_points_global.reserve(
        static_cast<size_t>(s_i.cached_frontier_marker_points_global.cols()) *
        2);

    std::vector<Eigen::Array2Xi> indices_in_updated_submaps;
    indices_in_updated_submaps.reserve(updated_submap_ids->size());

    std::vector<Submap*> updated_submaps;
    updated_submaps.reserve(updated_submap_ids->size());
    for (int i = 0; i < static_cast<int>(updated_submap_ids->size()); i++) {
      updated_submaps.push_back(&submaps_((*updated_submap_ids)[i]));
      indices_in_updated_submaps.push_back(
          (((updated_submaps[i]->to_local_submap_position *
             s_i.cached_frontier_marker_points_global)
                .array()
                .colwise() -
            updated_submaps[i]->limits().max().array()) /
               (-updated_submaps[i]->limits().resolution()) -
           0.5)
              .round()
              .cast<int>());
    }

    for (int j = 0; j < s_i.cached_frontier_marker_points_global.cols(); j++) {
      const auto global_position =
          s_i.cached_frontier_marker_points_global.col(j);

      bool ok = true;

      for (int i = 0; i < static_cast<int>(updated_submap_ids->size()); i++) {
        ok &= updated_submaps[i]->is_unknown(
            {indices_in_updated_submaps[i](1, j),
             indices_in_updated_submaps[i](0, j)});
      }

      if (ok) {
        updated_frontier_marker_points_global.insert(
            updated_frontier_marker_points_global.end(),
            {global_position.x(), global_position.y()});
        geometry_msgs::Point frontier_point;
        frontier_point.x = global_position.x();
        frontier_point.y = global_position.y();
        frontier_point.z = 0.;  // global_position.z();
        s_i.frontier_marker.points.push_back(frontier_point);
        // submap_hint = {-1, -1};
      }
    }
  }
  s_i.cached_frontier_marker_points_global = Eigen::Map<Eigen::Matrix2Xd>(
      updated_frontier_marker_points_global.data(), 2,
      updated_frontier_marker_points_global.size() / 2);

  return s_i.frontier_marker;
}

std::vector<cartographer::mapping::SubmapId>
Detector::GetIntersectingFinishedSubmaps(
    const cartographer::mapping::SubmapId& id_i) {
  std::vector<Value> intersecting_submaps;
  rt_.query(bgi::intersects(bounding_boxes_.at(id_i).last_global_box),
            std::back_inserter(intersecting_submaps));
  std::vector<cartographer::mapping::SubmapId> result;
  result.reserve(intersecting_submaps.size());
  for (const auto& intersecting_submap : intersecting_submaps) {
    result.push_back(intersecting_submap.second);
  }
  return result;
}

void Detector::HandleSubmapUpdates(
    const std::vector<cartographer::mapping::SubmapId>& submap_ids) {
  bool do_not_skip = false;
  total_submap_updates++;
  std::vector<cartographer::mapping::PoseGraphInterface::SubmapData>
      submap_data(submap_ids.size());
  for (int i = 0; i < static_cast<int>(submap_ids.size()); i++) {
    const auto& id_i = submap_ids[i];
    submap_data[i] = pose_graph_->GetSubmapData(id_i);
    if (submap_data[i].submap->insertion_finished()) {
      do_not_skip = true;
    }
  }

  if (!do_not_skip && lambda_worker_.TaskCount() > 10) {
    skipped_updates++;
    return;
  }

  auto* submap_copies_ptr = new std::vector<
      std::pair<cartographer::mapping::PoseGraphInterface::SubmapData,
                std::unique_ptr<cartographer::mapping::ProbabilityGrid>>>(
      submap_ids.size());
  for (int i = 0; i < static_cast<int>(submap_ids.size()); i++) {
    (*submap_copies_ptr)[i] = std::make_pair(
        submap_data[i],
        absl::make_unique<cartographer::mapping::ProbabilityGrid>(
            *static_cast<const cartographer::mapping::ProbabilityGrid*>(
                static_cast<const cartographer::mapping::Submap2D*>(
                    submap_data[i].submap.get())
                    ->grid())));
  }

  lambda_worker_.PushIntoWorkQueue([this, submap_copies_ptr, submap_ids]() {
    std::unique_ptr<std::vector<
        std::pair<cartographer::mapping::PoseGraphInterface::SubmapData,
                  std::unique_ptr<cartographer::mapping::ProbabilityGrid>>>>
        submap_copies(submap_copies_ptr);

    std::vector<cartographer::mapping::SubmapId> additional_submaps_to_publish;

    for (int i = 0; i < static_cast<int>(submap_ids.size()); i++) {
      submaps_.UpdateCacheWithCopy(submap_ids[i],
                                   std::move((*submap_copies)[i].first),
                                   std::move((*submap_copies)[i].second));
    }

    for (const auto& id_i : submap_ids) {
      const Submap& s_i(submaps_(id_i));
      CHECK(s_i.is_copy);

      int sum = 0;
      for (auto& cost : s_i.grid().correspondence_cost_cells()) {
        if (cost != 0) sum++;
      }

      auto& submap_frontier_cells = submap_frontier_points_[id_i];

      int previous_frontier_size =
          static_cast<int>(submap_frontier_cells.second.size());
      submap_frontier_cells.second.clear();

      Eigen::Array2i offset;
      cartographer::mapping::CellLimits cell_limits;
      s_i.grid().ComputeCroppedLimits(&offset, &cell_limits);

      const int full_x_dim = s_i.limits().cell_limits().num_x_cells;
      const int full_y_dim = s_i.limits().cell_limits().num_y_cells;

      using DynamicArray =
          Eigen::Array<uint16_t, Eigen::Dynamic, Eigen::Dynamic>;
      Eigen::Map<const DynamicArray> full_correspondence_costs(
          s_i.grid().correspondence_cost_cells().data(), full_x_dim,
          full_y_dim);

      const int x_dim = cell_limits.num_x_cells;
      const int y_dim = cell_limits.num_y_cells;

      DynamicArray correspondence_costs(x_dim + 2, y_dim + 2);
      correspondence_costs.row(0) = 0;
      correspondence_costs.row(x_dim + 1) = 0;
      correspondence_costs.col(0) = 0;
      correspondence_costs.col(y_dim + 1) = 0;

      correspondence_costs.block(1, 1, x_dim, y_dim) =
          full_correspondence_costs.block(offset.x(), offset.y(), x_dim, y_dim);

      DynamicArray free_cells(
          (correspondence_costs >= kFreeProbabilityValue).cast<uint16_t>());
      DynamicArray unknown_cells(
          ((correspondence_costs == 0) +
           (correspondence_costs > kOccupiedProbabilityValue) *
               (correspondence_costs < kFreeProbabilityValue))
              .cast<uint16_t>());

      DynamicArray free_neighbours = free_cells.block(0, 0, x_dim, y_dim) +
                                     free_cells.block(0, 1, x_dim, y_dim) +
                                     free_cells.block(0, 2, x_dim, y_dim) +
                                     free_cells.block(1, 2, x_dim, y_dim) +
                                     free_cells.block(2, 2, x_dim, y_dim) +
                                     free_cells.block(2, 1, x_dim, y_dim) +
                                     free_cells.block(2, 0, x_dim, y_dim) +
                                     free_cells.block(1, 0, x_dim, y_dim);

      DynamicArray unknown_neighbours =
          unknown_cells.block(0, 0, x_dim, y_dim) +
          unknown_cells.block(0, 1, x_dim, y_dim) +
          unknown_cells.block(0, 2, x_dim, y_dim) +
          unknown_cells.block(1, 2, x_dim, y_dim) +
          unknown_cells.block(2, 2, x_dim, y_dim) +
          unknown_cells.block(2, 1, x_dim, y_dim) +
          unknown_cells.block(2, 0, x_dim, y_dim) +
          unknown_cells.block(1, 0, x_dim, y_dim);

      DynamicArray frontier(unknown_cells.block(1, 1, x_dim, y_dim) *
                            (unknown_neighbours >= 3u).cast<uint16_t>() *
                            (free_neighbours >= 3u).cast<uint16_t>());

      std::vector<Submap*> previous_submaps_to_cleanup;
      for (int i = 1; i <= 2; i++) {
        const cartographer::mapping::SubmapId id_prev(
            {id_i.trajectory_id, id_i.submap_index - i});
        auto* submap_prev = submaps_.IfExists(id_prev);
        if (submap_prev != nullptr) {
          previous_submaps_to_cleanup.push_back(submap_prev);
        }
      }

      std::vector<int> frontier_cell_indexes_vec;
      frontier_cell_indexes_vec.reserve(
          static_cast<size_t>(previous_frontier_size) * 2);
      for (int y = 0; y < y_dim; y++)
        for (int x = 0; x < x_dim; x++)
          if (frontier(x, y))
            frontier_cell_indexes_vec.insert(frontier_cell_indexes_vec.end(),
                                             {y, x});

      const int num_frontier_candidates =
          static_cast<int>(frontier_cell_indexes_vec.size()) / 2;

      Eigen::Array2Xi frontier_cell_indexes = Eigen::Map<Eigen::Matrix2Xi>(
          frontier_cell_indexes_vec.data(), 2, num_frontier_candidates);

      frontier_cell_indexes.colwise() += Eigen::Array2i{offset.y(), offset.x()};

      Eigen::Array2Xd frontier_points(2, num_frontier_candidates);
      frontier_points.colwise() =
          Eigen::Array2d(s_i.limits().max().x(), s_i.limits().max().y());
      frontier_points -= (s_i.limits().resolution()) *
                         (frontier_cell_indexes.cast<double>() + 0.5);

      std::vector<bool> ok(static_cast<size_t>(num_frontier_candidates), true);
      // Perform stabbing query tests of current submap's s_i frontier
      // cells against temporally previous submaps
      int final_num_frontier_cells = num_frontier_candidates;
      for (const auto& previous_submap : previous_submaps_to_cleanup) {
        Eigen::Array2Xi frontier_cell_2_indexes =
            ((frontier_points.colwise() -
              previous_submap->limits().max().array()) /
                 (-previous_submap->limits().resolution()) -
             0.5)
                .round()
                .cast<int>();

        for (int i = 0; i < num_frontier_candidates; i++) {
          Eigen::Array2i xy_index{frontier_cell_2_indexes(1, i),
                                  frontier_cell_2_indexes(0, i)};
          const bool is_unknown = previous_submap->is_unknown(xy_index);
          if (ok[i] && !is_unknown) final_num_frontier_cells--;
          ok[i] = ok[i] && is_unknown;
        }
      }

      std::vector<double> final_frontier_points_vec;
      final_frontier_points_vec.reserve(
          static_cast<size_t>(final_num_frontier_cells) * 3);
      for (int i = 0; i < num_frontier_candidates; i++) {
        if (ok[i]) {
          final_frontier_points_vec.insert(
              final_frontier_points_vec.end(),
              {frontier_points(0, i), frontier_points(1, i), 1.});
        }
      }

      submap_frontier_cells.first = Eigen::Map<Eigen::Matrix3Xd>(
          final_frontier_points_vec.data(), 3, final_num_frontier_cells);
      submap_frontier_cells.second = {
          static_cast<size_t>(final_num_frontier_cells),
          cartographer::mapping::SubmapId{-1, -1}};

      const double max_x =
          s_i.limits().max().x() - s_i.limits().resolution() * offset.y();
      const double max_y =
          s_i.limits().max().y() - s_i.limits().resolution() * offset.x();
      const double min_x =
          s_i.limits().max().x() -
          s_i.limits().resolution() * (offset.y() + cell_limits.num_y_cells);
      const double min_y =
          s_i.limits().max().y() -
          s_i.limits().resolution() * (offset.x() + cell_limits.num_x_cells);

      const Eigen::Vector3d p1 =
          s_i.local_pose_inverse * Eigen::Vector3d{max_x, max_y, 0.};
      const Eigen::Vector3d p2 =
          s_i.local_pose_inverse * Eigen::Vector3d{min_x, min_y, 0.};
      auto& bounding_box_info = bounding_boxes_[s_i.id];
      bounding_box_info.local_box = std::make_pair(p1, p2);
      bounding_box_info.last_global_box = CalculateBoundingBox(s_i);

      for (const auto& previous_submap : previous_submaps_to_cleanup) {
        // Perform testing of intersecting submaps' frontier points against
        // the active submap
        auto& previous_submap_frontier_points =
            submap_frontier_points_.at(previous_submap->id);
        const auto submap_frontier_points_to_cleanup =
            std::move(previous_submap_frontier_points);
        int num_frontier_cells_to_clean =
            static_cast<int>(submap_frontier_points_to_cleanup.second.size());

        Eigen::Array2Xi frontier_cell_2_indexes =
            ((submap_frontier_points_to_cleanup.first.array()
                  .topRows(2)
                  .colwise() -
              s_i.limits().max().array()) /
                 (-s_i.limits().resolution()) -
             0.5)
                .round()
                .cast<int>();

        std::vector<double> final_cleaned_frontier_points_vec;
        final_cleaned_frontier_points_vec.reserve(
            static_cast<size_t>(num_frontier_cells_to_clean) * 3);
        for (int i = 0; i < num_frontier_cells_to_clean; i++) {
          Eigen::Array2i xy_index{frontier_cell_2_indexes(1, i),
                                  frontier_cell_2_indexes(0, i)};
          if (s_i.is_unknown(xy_index)) {
            final_cleaned_frontier_points_vec.insert(
                final_cleaned_frontier_points_vec.end(),
                {submap_frontier_points_to_cleanup.first(0, i),
                 submap_frontier_points_to_cleanup.first(1, i), 1.});
            previous_submap_frontier_points.second.push_back(
                submap_frontier_points_to_cleanup.second[i]);
          }
        }

        previous_submap_frontier_points.first = Eigen::Map<Eigen::Matrix3Xd>(
            final_cleaned_frontier_points_vec.data(), 3,
            previous_submap_frontier_points.second.size());

        if (std::find(additional_submaps_to_publish.begin(),
                      additional_submaps_to_publish.end(),
                      previous_submap->id) ==
            additional_submaps_to_publish.end())
          additional_submaps_to_publish.push_back(previous_submap->id);
      }

      // Keep only finished submaps in the tree in order to avoid lots of
      // insertions and removals while the submaps are being built due to the
      // bounding box being expanded.
      const auto iter =
          std::find(active_submaps_.begin(), active_submaps_.end(), s_i.id);
      if (s_i.finished) {
        // LOG(WARNING) << "Removing from active submaps: " << s_i.id;
        if (iter != active_submaps_.end()) active_submaps_.erase(iter);
        rt_.insert(std::make_pair(bounding_box_info.last_global_box, s_i.id));
      } else {
        if (iter == active_submaps_.end()) {
          active_submaps_.push_back(s_i.id);
        }
      }

      const auto intersecting_submaps = GetIntersectingFinishedSubmaps(id_i);
      for (const auto& intersecting_submap : intersecting_submaps) {
        if (std::find(additional_submaps_to_publish.begin(),
                      additional_submaps_to_publish.end(),
                      intersecting_submap) ==
            additional_submaps_to_publish.end())
          additional_submaps_to_publish.push_back(intersecting_submap);
      }
    }

    if (!CheckForOptimizationEvent()) {
      PublishSubmaps(submap_ids, additional_submaps_to_publish);
    }
  });
}

void Detector::RebuildTree() {
  std::unique_lock<std::mutex> lock(mutex_);
  std::vector<Value> rectangles;

  for (const auto& submap_data : submaps_.last_all_submap_data()) {
    const auto bounding_box_iter = bounding_boxes_.find(submap_data.id);
    if (bounding_box_iter == bounding_boxes_.end()) {
      continue;
    }
    auto& bounding_box_info = bounding_box_iter->second;
    const Submap& s_i(submaps_(submap_data.id));
    bounding_box_info.last_global_box = CalculateBoundingBox(s_i);

    if (std::find(active_submaps_.begin(), active_submaps_.end(), s_i.id) ==
        active_submaps_.end()) {
      rectangles.emplace_back(
          std::make_pair(bounding_box_info.last_global_box, s_i.id));
    }
  }

  // Invokes rtree's packing constructor.
  rt_ = RTree{std::move(rectangles)};
}

}  // namespace frontier
