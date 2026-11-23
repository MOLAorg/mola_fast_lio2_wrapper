// SPDX-License-Identifier: BSD-3-Clause
// Copyright (C) 2026, Jose Luis Blanco-Claraco
// See LICENSES/BSD-3-Clause.txt and THIRD_PARTY_NOTICES.md.

/**
 * @file   Odometry.h
 * @brief  Runs fast_lio2_core (GPL-2.0-only) on MRPT observations.
 *
 * Only MRPT types cross this interface, so that including it needs neither
 * PCL nor the fast_lio2_core headers.
 */
#pragma once

#include <mrpt/containers/yaml.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DInterpolator.h>
#include <mrpt/system/COutputLogger.h>

#include <memory>
#include <optional>
#include <string>

namespace fast_lio2_mrpt
{
/** Fast-LIO2 LiDAR-inertial odometry fed with MRPT observations.
 *
 * Takes `mrpt::obs::CObservationIMU` and `mrpt::obs::CObservationPointCloud`
 * observations, told apart by their sensor labels, and accumulates the
 * estimated trajectory of the output frame (`base_link`, or the LiDAR frame
 * when `baselink2lidar_pose_str` is left at identity).
 *
 * Every call to processObservation() runs to completion before it returns,
 * so no observation is ever dropped.
 */
class Odometry : public mrpt::system::COutputLogger
{
public:
  Odometry();
  ~Odometry() override;

  Odometry(const Odometry &) = delete;
  Odometry & operator=(const Odometry &) = delete;

  /** Reads the algorithm parameters (see the YAML files in `pipelines/`) and creates the
   * filter. Must be called once, before any observation. */
  void initialize(const mrpt::containers::yaml & cfg);

  /** Feeds one observation. Those whose sensor label matches neither
   * `imu_sensor_label` nor `lidar_sensor_label` are ignored. An error while
   * processing a LiDAR scan is logged and that scan skipped. */
  void processObservation(const mrpt::obs::CObservation::ConstPtr & obs);

  /** Trajectory estimated so far, in the output frame. */
  const mrpt::poses::CPose3DInterpolator & trajectory() const;

  /** The file the pipeline asks the trajectory to be saved to
   * (`save_trajectory_to_file` and `trajectory_output_file`), if any. */
  std::optional<std::string> trajectoryOutputFile() const;

  /** The pose this class reports, from the one the filter estimates.
   *
   * Fast-LIO2's state is the IMU pose in the world frame; the output frame
   * is `base_link`. Both extrinsics are given in the "wrt" direction the
   * pipeline YAMLs use, and both have to be inverted to get there:
   *
   *   T_w_base = T_w_imu . T_lidar_imu^-1 . T_base_lidar^-1
   *
   * @param worldFromImu        what the filter estimates, `T_w_imu`.
   * @param imuPoseInLidar      `T_lidar_imu`, YAML `imu_pose_in_lidar`.
   * @param lidarPoseInBaseLink `T_base_lidar`, YAML `baselink2lidar_pose_str`.
   */
  static mrpt::poses::CPose3D worldFromBaseLink(
    const mrpt::poses::CPose3D & worldFromImu, const mrpt::poses::CPose3D & imuPoseInLidar,
    const mrpt::poses::CPose3D & lidarPoseInBaseLink);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;

  void processImu(const mrpt::obs::CObservation::ConstPtr & obs);
  void processLidarScan(const mrpt::obs::CObservation::ConstPtr & obs);
  void collectOutputs();
};

}  // namespace fast_lio2_mrpt
