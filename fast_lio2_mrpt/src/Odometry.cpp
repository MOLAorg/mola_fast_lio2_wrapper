// SPDX-License-Identifier: BSD-3-Clause
// Copyright (C) 2026, Jose Luis Blanco-Claraco
// See LICENSES/BSD-3-Clause.txt and THIRD_PARTY_NOTICES.md.

/**
 * @file   Odometry.cpp
 * @brief  Runs fast_lio2_core (GPL-2.0-only) on MRPT observations.
 */

// clang-format off
// Must come first: it includes PCL before the MRPT headers below.
#include "Adapters.h"
// clang-format on

#include <fast_lio2_core/FastLio2Core.h>
#include <fast_lio2_mrpt/Odometry.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>

#include <regex>
#include <sstream>

namespace fast_lio2_mrpt
{
namespace
{
/// Parses "x y z yaw_deg pitch_deg roll_deg".
mrpt::poses::CPose3D parsePose6D(const std::string & s)
{
  std::istringstream iss(s);
  double x = 0;
  double y = 0;
  double z = 0;
  double yaw = 0;
  double pitch = 0;
  double roll = 0;
  if (!(iss >> x >> y >> z >> yaw >> pitch >> roll)) {
    THROW_EXCEPTION_FMT(
      "Could not parse pose string '%s' (expected: 'x y z yaw_deg pitch_deg roll_deg')", s.c_str());
  }
  return {x, y, z, mrpt::DEG2RAD(yaw), mrpt::DEG2RAD(pitch), mrpt::DEG2RAD(roll)};
}

mrpt::poses::CPose3D isometryToPose(const Eigen::Isometry3d & iso)
{
  mrpt::math::CMatrixDouble44 m;
  for (int r = 0; r < 4; r++) {
    for (int c = 0; c < 4; c++) {
      m(r, c) = iso.matrix()(r, c);
    }
  }
  return mrpt::poses::CPose3D(m);
}
}  // namespace

struct Odometry::Impl
{
  std::regex lidar_label_regex{"lidar"};
  std::regex imu_label_regex{"imu"};

  /// `T_lidar_imu`: pose of the IMU frame wrt the LiDAR frame (the same
  /// convention dataset YAMLs use for an IMU's `fixed_sensor_pose`).
  mrpt::poses::CPose3D imu_pose_in_lidar;

  /// `T_base_lidar`: pose of the LiDAR frame wrt `base_link`. Identity when
  /// the LiDAR itself is the reference frame.
  mrpt::poses::CPose3D lidar_pose_in_base_link;

  /// `T_imu_lidar . T_lidar_base`, precomputed: right-compose the filter's
  /// own `T_w_imu` with it to get the reported pose (see worldFromBaseLink()).
  mrpt::poses::CPose3D imu_to_output_frame;

  /// Scan duration assumed for clouds without a per-point time channel.
  double fallback_scan_period = 0.1;

  /// Treat every point of a scan as taken at the observation timestamp, i.e.
  /// no per-point de-skew even when the cloud carries a time channel: for
  /// streams already motion-compensated whose per-point times still span the
  /// sweep, de-skewing again distorts the geometry.
  bool ignore_point_timestamps = false;

  bool save_trajectory_to_file = false;
  std::string trajectory_output_file;

  fast_lio2_core::Config core_cfg;
  std::unique_ptr<fast_lio2_core::FastLio2Core> core;

  mrpt::poses::CPose3DInterpolator trajectory;
};

Odometry::Odometry() : mrpt::system::COutputLogger("FastLio2"), impl_(std::make_unique<Impl>()) {}

Odometry::~Odometry() = default;

void Odometry::initialize(const mrpt::containers::yaml & cfg)
{
  MRPT_START

  auto & p = *impl_;

  const auto lidarLabel = cfg.getOrDefault<std::string>("lidar_sensor_label", "lidar");
  const auto imuLabel = cfg.getOrDefault<std::string>("imu_sensor_label", "imu");
  p.lidar_label_regex = std::regex(lidarLabel);
  p.imu_label_regex = std::regex(imuLabel);

  p.imu_pose_in_lidar =
    parsePose6D(cfg.getOrDefault<std::string>("imu_pose_in_lidar", "0 0 0 0 0 0"));
  p.lidar_pose_in_base_link =
    parsePose6D(cfg.getOrDefault<std::string>("baselink2lidar_pose_str", "0 0 0 0 0 0"));

  p.fallback_scan_period = cfg.getOrDefault<double>("fallback_scan_period", p.fallback_scan_period);
  p.ignore_point_timestamps =
    cfg.getOrDefault<bool>("ignore_point_timestamps", p.ignore_point_timestamps);
  p.save_trajectory_to_file =
    cfg.getOrDefault<bool>("save_trajectory_to_file", p.save_trajectory_to_file);
  p.trajectory_output_file =
    cfg.getOrDefault<std::string>("trajectory_output_file", p.trajectory_output_file);

  auto & c = p.core_cfg;
  c.gyr_cov = cfg.getOrDefault<double>("gyr_cov", c.gyr_cov);
  c.acc_cov = cfg.getOrDefault<double>("acc_cov", c.acc_cov);
  c.b_gyr_cov = cfg.getOrDefault<double>("b_gyr_cov", c.b_gyr_cov);
  c.b_acc_cov = cfg.getOrDefault<double>("b_acc_cov", c.b_acc_cov);
  c.filter_size_surf = cfg.getOrDefault<double>("filter_size_surf", c.filter_size_surf);
  c.filter_size_map = cfg.getOrDefault<double>("filter_size_map", c.filter_size_map);
  c.cube_side_length = cfg.getOrDefault<double>("cube_side_length", c.cube_side_length);
  c.det_range = cfg.getOrDefault<float>("det_range", c.det_range);
  c.max_iteration = cfg.getOrDefault<int>("max_iteration", c.max_iteration);
  c.extrinsic_est_en = cfg.getOrDefault<bool>("extrinsic_est_en", c.extrinsic_est_en);
  c.init_time = cfg.getOrDefault<double>("init_time", c.init_time);
  c.blind = cfg.getOrDefault<double>("blind", c.blind);

  // Fast-LIO2's extrinsic_R/T are "LiDAR wrt IMU" (p_imu = R*p_lidar + T),
  // the inverse of `imu_pose_in_lidar`.
  mrpt::poses::CPose3D lidarPoseInImu = p.imu_pose_in_lidar;
  lidarPoseInImu.inverse();
  c.extrinsic_R = lidarPoseInImu.getRotationMatrix().asEigen();
  c.extrinsic_T = Eigen::Vector3d(lidarPoseInImu.x(), lidarPoseInImu.y(), lidarPoseInImu.z());

  p.imu_to_output_frame = worldFromBaseLink(
    mrpt::poses::CPose3D::Identity(), p.imu_pose_in_lidar, p.lidar_pose_in_base_link);

  p.core = std::make_unique<fast_lio2_core::FastLio2Core>(c);

  MRPT_LOG_INFO_STREAM(
    "Fast-LIO2 initialized. lidar_sensor_label='" << lidarLabel << "' imu_sensor_label='"
                                                  << imuLabel << "'");

  MRPT_END
}

void Odometry::processObservation(const mrpt::obs::CObservation::ConstPtr & obs)
{
  MRPT_START
  ASSERT_(obs);

  if (std::regex_match(obs->sensorLabel, impl_->imu_label_regex)) {
    processImu(obs);
  } else if (std::regex_match(obs->sensorLabel, impl_->lidar_label_regex)) {
    // A failed scan must not end a long run: report it and go on.
    try {
      processLidarScan(obs);
    } catch (const std::exception & e) {
      MRPT_LOG_ERROR_STREAM("Error processing a LiDAR scan: " << e.what());
    } catch (...) {
      MRPT_LOG_ERROR("Unknown error processing a LiDAR scan");
    }
  }

  MRPT_END
}

void Odometry::processImu(const mrpt::obs::CObservation::ConstPtr & obs)
{
  const auto imu = std::dynamic_pointer_cast<const mrpt::obs::CObservationIMU>(obs);
  if (!imu || !impl_->core) {
    return;
  }
  impl_->core->addImu(toFastLio2Imu(*imu));
  collectOutputs();
}

void Odometry::processLidarScan(const mrpt::obs::CObservation::ConstPtr & obs)
{
  const auto pc = std::dynamic_pointer_cast<const mrpt::obs::CObservationPointCloud>(obs);
  if (!pc || !impl_->core) {
    return;
  }
  auto & p = *impl_;
  double tBegin = 0;
  double tEnd = 0;
  auto cloud = toFastLio2Cloud(
    *pc, p.fallback_scan_period, tBegin, tEnd, p.core_cfg.blind, p.ignore_point_timestamps);
  p.core->addLidarScan(std::move(cloud), tBegin, tEnd);
  collectOutputs();
}

void Odometry::collectOutputs()
{
  auto & p = *impl_;
  for (const auto & out : p.core->drainReady()) {
    // The filter estimates the IMU pose; report the output frame instead.
    const mrpt::poses::CPose3D imuPose = isometryToPose(out.pose_imu);
    const mrpt::poses::CPose3D outputPose = imuPose + p.imu_to_output_frame;
    p.trajectory.insert(mrpt::Clock::fromDouble(out.timestamp), outputPose);
  }
}

const mrpt::poses::CPose3DInterpolator & Odometry::trajectory() const { return impl_->trajectory; }

std::optional<std::string> Odometry::trajectoryOutputFile() const
{
  if (!impl_->save_trajectory_to_file || impl_->trajectory_output_file.empty()) {
    return std::nullopt;
  }
  return impl_->trajectory_output_file;
}

mrpt::poses::CPose3D Odometry::worldFromBaseLink(
  const mrpt::poses::CPose3D & worldFromImu, const mrpt::poses::CPose3D & imuPoseInLidar,
  const mrpt::poses::CPose3D & lidarPoseInBaseLink)
{
  mrpt::poses::CPose3D lidarPoseInImu = imuPoseInLidar;
  lidarPoseInImu.inverse();
  mrpt::poses::CPose3D baseLinkPoseInLidar = lidarPoseInBaseLink;
  baseLinkPoseInLidar.inverse();
  return worldFromImu + lidarPoseInImu + baseLinkPoseInLidar;
}

}  // namespace fast_lio2_mrpt
