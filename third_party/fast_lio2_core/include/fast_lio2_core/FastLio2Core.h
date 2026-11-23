// SPDX-License-Identifier: GPL-2.0-only AND BSD-3-Clause
//
// Derived from Fast-LIO2's laserMapping.cpp
// (https://github.com/hku-mars/FAST_LIO, branch ROS2), GNU GPL v2.0-only,
// Copyright HKU-MARS lab (algorithm), with inherited LOAM-lineage notices
// (Ji Zhang/CMU, Southwest Research Institute, Livox) in the original file.
// The per-scan processing steps (`lasermap_fov_segment`, `h_share_model`,
// `map_incremental`, and the main loop body) are transplanted here from
// free functions and global state into a stateful class with a push-based
// API, replacing ROS pub/sub and the `sync_packages()` buffer-polling loop.
// See THIRD_PARTY_NOTICES.md at the repository root.
//
// Notice retained, verbatim, from the original laserMapping.cpp:
//
// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//
// Modifier: Livox               dev@livoxtech.com
//
// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <fast_lio2_core/ImuProcess.h>
#include <fast_lio2_core/Types.hpp>

#include <ikd_Tree.h>
#include <pcl/filters/voxel_grid.h>

#include <deque>
#include <optional>
#include <vector>

namespace fast_lio2_core
{
/// Fast-LIO2's algorithm core: IMU-propagated, iterated-ESKF LiDAR-inertial
/// odometry with an incremental ikd-Tree map. No ROS, no MOLA dependency.
///
/// Usage: call `addImu()` for every IMU sample and `addLidarScan()` for
/// every LiDAR scan, in timestamp order (interleaving is fine, and neither
/// call needs to happen from the same thread). Call `drainReady()`
/// afterwards to collect any newly-completed scan results: a scan is only
/// ever processed once enough IMU data covering its full time span has been
/// received, so `addLidarScan()` may not resolve immediately if IMU data is
/// still catching up -- **do not treat that as a dropped scan**, it is
/// picked up automatically by a later `addImu()`/`drainReady()` call.
class FastLio2Core
{
 public:
  explicit FastLio2Core(const Config& cfg);

  /// Buffers an IMU sample and, if a pending scan now has full IMU coverage,
  /// processes it (results collected via `drainReady()`). Cheap; never
  /// drops a sample.
  void addImu(const ImuSample& s);

  /// Buffers a LiDAR scan (points in LiDAR body frame) and processes it
  /// immediately if enough IMU data is already available.
  void addLidarScan(LidarPointVector points, double t_beg, double t_end);

  /// Returns (and clears) every `OdometryOutput` produced since the last
  /// call. Usually 0 or 1 entries; can be more if several pending scans
  /// became ready at once.
  std::vector<OdometryOutput> drainReady();

  /// Snapshot of the incremental map maintained by the ikd-Tree, in world
  /// frame. May be expensive (flattens the whole tree); call sparingly.
  std::shared_ptr<const XYZIPointVector> currentMap() const;

  /// True once the estimator has left the initial IMU-alignment /
  /// early-scans warm-up window (mirrors upstream's `flg_EKF_inited`).
  bool isInitialized() const { return flg_ekf_inited_; }

  /// Only meant to be called by the h_share_model free-function trampoline.
  void hShareModel(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);

 private:
  struct PendingScan
  {
    LidarPointVector points;
    double            t_beg = 0;
    double            t_end = 0;
  };

  void tryProcessPending();
  bool processOneScan(const PendingScan& scan);

  void lasermapFovSegment();
  void mapIncremental();
  void pointBodyToWorld(const PointType& pi, PointType& po) const;

  static constexpr double LASER_POINT_COV  = 0.001;
  static constexpr float  MOV_THRESHOLD    = 1.5f;
  static constexpr float  LIDAR_SP_LEN     = 2.0f;

  Config cfg_;

  std::deque<ImuSample>   imu_buffer_;
  std::deque<PendingScan> pending_scans_;
  std::vector<OdometryOutput> ready_outputs_;

  ImuProcess                                    imu_proc_;
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
  // Heap-allocated: KD_TREE's background-rebuild logger is a ~60-100 MB
  // fixed-size array member (`Operation_Logger_Type q[Q_LEN]`, Q_LEN =
  // 1'000'000), so a by-value KD_TREE member would overflow the stack the
  // moment any class embedding it (this one) is stack-constructed.
  std::shared_ptr<KD_TREE<PointType>> ikdtree_ = std::make_shared<KD_TREE<PointType>>();
  pcl::VoxelGrid<PointType>                     down_size_filter_surf_;

  state_ikfom     state_point_;
  Eigen::Vector3d pos_lid_ = Eigen::Vector3d::Zero();

  BoxPointType local_map_points_{};
  bool         localmap_initialized_ = false;
  bool         flg_first_scan_       = true;
  bool         flg_ekf_inited_       = false;
  double       first_lidar_time_     = 0;

  MeasureGroup measures_;

  PointCloudXYZI::Ptr feats_undistort_{new PointCloudXYZI()};
  PointCloudXYZI::Ptr feats_down_body_{new PointCloudXYZI()};
  PointCloudXYZI::Ptr feats_down_world_{new PointCloudXYZI()};
  PointCloudXYZI::Ptr normvec_{new PointCloudXYZI()};
  PointCloudXYZI::Ptr laser_cloud_ori_{new PointCloudXYZI()};
  PointCloudXYZI::Ptr corr_normvect_{new PointCloudXYZI()};

  int feats_down_size_ = 0;
  int effct_feat_num_  = 0;

  std::vector<bool>                          point_selected_surf_;
  std::vector<float>                         res_last_;
  std::vector<KD_TREE<PointType>::PointVector> nearest_points_;
};

}  // namespace fast_lio2_core
