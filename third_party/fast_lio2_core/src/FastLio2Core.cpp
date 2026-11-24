// SPDX-License-Identifier: GPL-2.0-only AND BSD-3-Clause
//
// Derived from Fast-LIO2's laserMapping.cpp and common_lib.h
// (https://github.com/hku-mars/FAST_LIO, branch ROS2), GNU GPL v2.0-only,
// Copyright HKU-MARS lab (algorithm), with inherited LOAM-lineage notices
// (Ji Zhang/CMU, Southwest Research Institute, Livox) in the original file.
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
//
// PCL_NO_PRECOMPILE: forces pcl::VoxelGrid<PointType> to be instantiated
// from headers in this translation unit instead of linking the symbol
// precompiled into the system libpcl_filters.so. On at least one dev
// machine, the system PCL package's precompiled VoxelGrid<PointXYZINormal>
// was built against a subtly different Eigen aligned-allocator ABI than the
// currently-installed Eigen headers, causing a heap-buffer-overflow on
// deallocation across the .so boundary. Header-only instantiation sidesteps
// that ABI mismatch entirely.
#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
#include <fast_lio2_core/FastLio2Core.h>
#include <fast_lio2_core/so3_math.h>

#include <cmath>
#include <fstream>

namespace fast_lio2_core {

namespace {
// Port of Fast-LIO2's common_lib.h `esti_plane()`: least-squares plane fit
// through `NUM_MATCH_POINTS` points, rejecting the fit if any point is
// further than `threshold` from the resulting plane.
bool esti_plane(Eigen::Vector4f &pca_result,
                const KD_TREE<PointType>::PointVector &points,
                float threshold) {
  Eigen::Matrix<float, NUM_MATCH_POINTS, 3> A;
  Eigen::Matrix<float, NUM_MATCH_POINTS, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    A(j, 0) = points[j].x;
    A(j, 1) = points[j].y;
    A(j, 2) = points[j].z;
  }

  Eigen::Vector3f normvec = A.colPivHouseholderQr().solve(b);
  const float n = normvec.norm();
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0f / n;

  for (int j = 0; j < NUM_MATCH_POINTS; j++) {
    if (std::fabs(pca_result(0) * points[j].x + pca_result(1) * points[j].y +
                  pca_result(2) * points[j].z + pca_result(3)) > threshold) {
      return false;
    }
  }
  return true;
}

float calc_dist(const PointType &p1, const PointType &p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) +
         (p1.z - p2.z) * (p1.z - p2.z);
}

// esekfom's `measurementModel_dyn_share` is a plain function pointer (not a
// std::function), so it cannot capture a `this`. A single "active instance"
// pointer, set only for the duration of one
// `update_iterated_dyn_share_modified()` call (never re-entrant: everything
// here runs synchronously within one `processOneScan()`), bridges the C-style
// callback to the owning instance.
thread_local FastLio2Core *g_active_core = nullptr;

void h_share_model_trampoline(
    state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  g_active_core->hShareModel(s, ekfom_data);
}
} // namespace

FastLio2Core::FastLio2Core(const Config &cfg) : cfg_(cfg) {
  imu_proc_.set_extrinsic(cfg_.extrinsic_T, cfg_.extrinsic_R);
  imu_proc_.set_gyr_cov(
      Eigen::Vector3d(cfg_.gyr_cov, cfg_.gyr_cov, cfg_.gyr_cov));
  imu_proc_.set_acc_cov(
      Eigen::Vector3d(cfg_.acc_cov, cfg_.acc_cov, cfg_.acc_cov));
  imu_proc_.set_gyr_bias_cov(
      Eigen::Vector3d(cfg_.b_gyr_cov, cfg_.b_gyr_cov, cfg_.b_gyr_cov));
  imu_proc_.set_acc_bias_cov(
      Eigen::Vector3d(cfg_.b_acc_cov, cfg_.b_acc_cov, cfg_.b_acc_cov));

  down_size_filter_surf_.setLeafSize(
      cfg_.filter_size_surf, cfg_.filter_size_surf, cfg_.filter_size_surf);

  double epsi[23];
  std::fill(epsi, epsi + 23, 0.001);
  kf_.init_dyn_share(get_f, df_dx, df_dw, h_share_model_trampoline,
                     cfg_.max_iteration, epsi);
}

void FastLio2Core::addImu(const ImuSample &s) {
  imu_buffer_.push_back(s);
  tryProcessPending();
}

void FastLio2Core::addLidarScan(LidarPointVector points, double t_beg,
                                double t_end) {
  pending_scans_.push_back(PendingScan{std::move(points), t_beg, t_end});
  tryProcessPending();
}

void FastLio2Core::tryProcessPending() {
  while (!pending_scans_.empty() && !imu_buffer_.empty() &&
         imu_buffer_.back().t >= pending_scans_.front().t_end) {
    PendingScan scan = std::move(pending_scans_.front());
    pending_scans_.pop_front();
    processOneScan(scan);
  }
}

bool FastLio2Core::processOneScan(const PendingScan &scan) {
  // Build the internal pcl::PointCloud here, inside fast_lio2_core's own
  // translation unit (see the ABI note on `PointCloudXYZI` in Types.hpp).
  auto cloud = std::make_shared<PointCloudXYZI>();
  cloud->points.resize(scan.points.size());
  for (size_t i = 0; i < scan.points.size(); i++) {
    const auto &sp = scan.points[i];
    auto &dp = cloud->points[i];
    dp.x = sp.x;
    dp.y = sp.y;
    dp.z = sp.z;
    dp.intensity = sp.intensity;
    dp.curvature = static_cast<float>(sp.t_offset_sec * 1000.0);
  }
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;

  measures_.lidar = std::move(cloud);
  measures_.lidar_beg_time = scan.t_beg;
  measures_.lidar_end_time = scan.t_end;
  measures_.imu.clear();
  while (!imu_buffer_.empty() && imu_buffer_.front().t <= scan.t_end) {
    measures_.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }
  // Keep the last-consumed IMU sample available as the "previous frame
  // tail" for the next scan's forward propagation (mirrors upstream's
  // `last_imu_` chaining in ImuProcess).
  if (!measures_.imu.empty())
    imu_buffer_.push_front(measures_.imu.back());

  if (flg_first_scan_) {
    first_lidar_time_ = measures_.lidar_beg_time;
    imu_proc_.first_lidar_time = first_lidar_time_;
    flg_first_scan_ = false;
    return false;
  }

  imu_proc_.Process(measures_, kf_, feats_undistort_);
  state_point_ = kf_.get_x();
  pos_lid_ = Eigen::Vector3d(state_point_.pos) +
             state_point_.rot * Eigen::Vector3d(state_point_.offset_T_L_I);

  if (!feats_undistort_ || feats_undistort_->empty())
    return false;

  flg_ekf_inited_ =
      (measures_.lidar_beg_time - first_lidar_time_) >= cfg_.init_time;

  lasermapFovSegment();

  down_size_filter_surf_.setInputCloud(feats_undistort_);
  down_size_filter_surf_.filter(*feats_down_body_);
  feats_down_size_ = static_cast<int>(feats_down_body_->points.size());

  if (ikdtree_->Root_Node == nullptr) {
    if (feats_down_size_ > 5) {
      ikdtree_->set_downsample_param(cfg_.filter_size_map);
      feats_down_world_->resize(feats_down_size_);
      for (int i = 0; i < feats_down_size_; i++) {
        pointBodyToWorld(feats_down_body_->points[i],
                         feats_down_world_->points[i]);
      }
      ikdtree_->Build(feats_down_world_->points);
    }
    return false;
  }

  if (feats_down_size_ < 5)
    return false;

  normvec_->resize(feats_down_size_);
  feats_down_world_->resize(feats_down_size_);
  point_selected_surf_.assign(feats_down_size_, true);
  res_last_.assign(feats_down_size_, -1000.f);
  nearest_points_.resize(feats_down_size_);

  double solve_H_time = 0;
  g_active_core = this;
  kf_.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
  g_active_core = nullptr;

  state_point_ = kf_.get_x();
  pos_lid_ = Eigen::Vector3d(state_point_.pos) +
             state_point_.rot * Eigen::Vector3d(state_point_.offset_T_L_I);

  mapIncremental();

  static bool MOLA_FASTLIO_DUMP_ESTIMATED_GRAV =
      ::getenv("MOLA_FASTLIO_DUMP_ESTIMATED_GRAV") != nullptr;
  if (MOLA_FASTLIO_DUMP_ESTIMATED_GRAV) {
    static std::ofstream f("grav.txt");
    char buf[100];
    sprintf(buf, "%.03f ", measures_.lidar_end_time);
    f << buf << state_point_.grav << "\n";
  }

  OdometryOutput out;
  out.timestamp = measures_.lidar_end_time;
  out.pose_imu.setIdentity();
  out.pose_imu.linear() = state_point_.rot.toRotationMatrix();
  out.pose_imu.translation() = state_point_.pos;
  out.vel = state_point_.vel;
  out.bias_gyro = state_point_.bg;
  out.bias_acc = state_point_.ba;
  out.gravity = state_point_.grav.get_vect();
  {
    auto pts = std::make_shared<XYZIPointVector>();
    pts->reserve(feats_down_body_->points.size());
    for (const auto &p : feats_down_body_->points) {
      pts->push_back(XYZIPoint{p.x, p.y, p.z, p.intensity});
    }
    out.deskewed_points_body = std::move(pts);
  }
  ready_outputs_.push_back(std::move(out));
  return true;
}

std::vector<OdometryOutput> FastLio2Core::drainReady() {
  std::vector<OdometryOutput> out;
  out.swap(ready_outputs_);
  return out;
}

void FastLio2Core::pointBodyToWorld(const PointType &pi, PointType &po) const {
  Eigen::Vector3d p_body(pi.x, pi.y, pi.z);
  Eigen::Vector3d p_global(state_point_.rot *
                               (state_point_.offset_R_L_I * p_body +
                                Eigen::Vector3d(state_point_.offset_T_L_I)) +
                           Eigen::Vector3d(state_point_.pos));
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

void FastLio2Core::lasermapFovSegment() {
  std::vector<BoxPointType> cub_needrm;

  const Eigen::Vector3d pos_lid = pos_lid_;
  if (!localmap_initialized_) {
    for (int i = 0; i < 3; i++) {
      local_map_points_.vertex_min[i] =
          pos_lid(i) - cfg_.cube_side_length / 2.0;
      local_map_points_.vertex_max[i] =
          pos_lid(i) + cfg_.cube_side_length / 2.0;
    }
    localmap_initialized_ = true;
    return;
  }

  float dist_to_map_edge[3][2];
  bool need_move = false;
  for (int i = 0; i < 3; i++) {
    dist_to_map_edge[i][0] =
        std::fabs(pos_lid(i) - local_map_points_.vertex_min[i]);
    dist_to_map_edge[i][1] =
        std::fabs(pos_lid(i) - local_map_points_.vertex_max[i]);
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * cfg_.det_range ||
        dist_to_map_edge[i][1] <= MOV_THRESHOLD * cfg_.det_range)
      need_move = true;
  }
  if (!need_move)
    return;

  BoxPointType new_local_map_points = local_map_points_;
  float mov_dist =
      std::max((cfg_.cube_side_length - 2.0 * MOV_THRESHOLD * cfg_.det_range) *
                   0.5 * 0.9,
               double(cfg_.det_range * (MOV_THRESHOLD - 1)));

  for (int i = 0; i < 3; i++) {
    BoxPointType tmp_boxpoints = local_map_points_;
    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * cfg_.det_range) {
      new_local_map_points.vertex_max[i] -= mov_dist;
      new_local_map_points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = local_map_points_.vertex_max[i] - mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * cfg_.det_range) {
      new_local_map_points.vertex_max[i] += mov_dist;
      new_local_map_points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = local_map_points_.vertex_min[i] + mov_dist;
      cub_needrm.push_back(tmp_boxpoints);
    }
  }
  local_map_points_ = new_local_map_points;

  KD_TREE<PointType>::PointVector points_history;
  ikdtree_->acquire_removed_points(points_history);

  if (!cub_needrm.empty())
    ikdtree_->Delete_Point_Boxes(cub_needrm);
}

void FastLio2Core::mapIncremental() {
  KD_TREE<PointType>::PointVector point_to_add;
  KD_TREE<PointType>::PointVector point_no_need_downsample;
  point_to_add.reserve(feats_down_size_);
  point_no_need_downsample.reserve(feats_down_size_);

  for (int i = 0; i < feats_down_size_; i++) {
    pointBodyToWorld(feats_down_body_->points[i], feats_down_world_->points[i]);

    if (!nearest_points_[i].empty() && flg_ekf_inited_) {
      const auto &points_near = nearest_points_[i];
      bool need_add = true;
      PointType mid_point;
      mid_point.x =
          std::floor(feats_down_world_->points[i].x / cfg_.filter_size_map) *
              cfg_.filter_size_map +
          0.5 * cfg_.filter_size_map;
      mid_point.y =
          std::floor(feats_down_world_->points[i].y / cfg_.filter_size_map) *
              cfg_.filter_size_map +
          0.5 * cfg_.filter_size_map;
      mid_point.z =
          std::floor(feats_down_world_->points[i].z / cfg_.filter_size_map) *
              cfg_.filter_size_map +
          0.5 * cfg_.filter_size_map;
      const float dist = calc_dist(feats_down_world_->points[i], mid_point);

      if (std::fabs(points_near[0].x - mid_point.x) >
              0.5 * cfg_.filter_size_map &&
          std::fabs(points_near[0].y - mid_point.y) >
              0.5 * cfg_.filter_size_map &&
          std::fabs(points_near[0].z - mid_point.z) >
              0.5 * cfg_.filter_size_map) {
        point_no_need_downsample.push_back(feats_down_world_->points[i]);
        continue;
      }

      for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
        if (static_cast<int>(points_near.size()) < NUM_MATCH_POINTS)
          break;
        if (calc_dist(points_near[readd_i], mid_point) < dist) {
          need_add = false;
          break;
        }
      }
      if (need_add)
        point_to_add.push_back(feats_down_world_->points[i]);
    } else {
      point_to_add.push_back(feats_down_world_->points[i]);
    }
  }

  ikdtree_->Add_Points(point_to_add, true);
  ikdtree_->Add_Points(point_no_need_downsample, false);
}

void FastLio2Core::hShareModel(
    state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data) {
  laser_cloud_ori_->clear();
  corr_normvect_->clear();

  for (int i = 0; i < feats_down_size_; i++) {
    PointType &point_body = feats_down_body_->points[i];
    PointType &point_world = feats_down_world_->points[i];

    Eigen::Vector3d p_body(point_body.x, point_body.y, point_body.z);
    Eigen::Vector3d p_global(
        s.rot * (s.offset_R_L_I * p_body + Eigen::Vector3d(s.offset_T_L_I)) +
        Eigen::Vector3d(s.pos));
    point_world.x = p_global(0);
    point_world.y = p_global(1);
    point_world.z = p_global(2);
    point_world.intensity = point_body.intensity;

    std::vector<float> point_search_sq_dis(NUM_MATCH_POINTS);
    auto &points_near = nearest_points_[i];

    if (ekfom_data.converge) {
      ikdtree_->Nearest_Search(point_world, NUM_MATCH_POINTS, points_near,
                               point_search_sq_dis);
      point_selected_surf_[i] =
          static_cast<int>(points_near.size()) < NUM_MATCH_POINTS ? false
          : point_search_sq_dis[NUM_MATCH_POINTS - 1] > 5         ? false
                                                                  : true;
    }

    if (!point_selected_surf_[i])
      continue;

    Eigen::Vector4f pabcd;
    point_selected_surf_[i] = false;
    if (esti_plane(pabcd, points_near, 0.1f)) {
      const float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                        pabcd(2) * point_world.z + pabcd(3);
      const float s2 = 1 - 0.9f * std::fabs(pd2) / std::sqrt(p_body.norm());

      if (s2 > 0.9f) {
        point_selected_surf_[i] = true;
        normvec_->points[i].x = pabcd(0);
        normvec_->points[i].y = pabcd(1);
        normvec_->points[i].z = pabcd(2);
        normvec_->points[i].intensity = pd2;
        res_last_[i] = std::fabs(pd2);
      }
    }
  }

  effct_feat_num_ = 0;
  double total_residual = 0.0;
  for (int i = 0; i < feats_down_size_; i++) {
    if (point_selected_surf_[i]) {
      laser_cloud_ori_->points.push_back(feats_down_body_->points[i]);
      corr_normvect_->points.push_back(normvec_->points[i]);
      total_residual += res_last_[i];
      effct_feat_num_++;
    }
  }
  (void)total_residual;

  if (effct_feat_num_ < 1) {
    ekfom_data.valid = false;
    return;
  }

  ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num_, 12);
  ekfom_data.h.resize(effct_feat_num_);

  for (int i = 0; i < effct_feat_num_; i++) {
    const PointType &laser_p = laser_cloud_ori_->points[i];
    Eigen::Vector3d point_this_be(laser_p.x, laser_p.y, laser_p.z);
    Eigen::Matrix3d point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
    Eigen::Vector3d point_this =
        s.offset_R_L_I * point_this_be + Eigen::Vector3d(s.offset_T_L_I);
    Eigen::Matrix3d point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    const PointType &norm_p = corr_normvect_->points[i];
    Eigen::Vector3d norm_vec(norm_p.x, norm_p.y, norm_p.z);

    Eigen::Vector3d C(s.rot.conjugate() * norm_vec);
    Eigen::Vector3d A(point_crossmat * C);
    if (cfg_.extrinsic_est_en) {
      Eigen::Vector3d B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, A(0),
          A(1), A(2), B(0), B(1), B(2), C(0), C(1), C(2);
    } else {
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, A(0),
          A(1), A(2), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }

    ekfom_data.h(i) = -norm_p.intensity;
  }
}

std::shared_ptr<const XYZIPointVector> FastLio2Core::currentMap() const {
  auto out = std::make_shared<XYZIPointVector>();
  if (ikdtree_->Root_Node != nullptr) {
    KD_TREE<PointType>::PointVector storage;
    ikdtree_->flatten(ikdtree_->Root_Node, storage, NOT_RECORD);
    out->reserve(storage.size());
    for (const auto &p : storage) {
      out->push_back(XYZIPoint{p.x, p.y, p.z, p.intensity});
    }
  }
  return out;
}

} // namespace fast_lio2_core
