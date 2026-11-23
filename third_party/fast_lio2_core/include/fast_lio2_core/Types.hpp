// SPDX-License-Identifier: GPL-2.0-only
//
// Plain data types used at the fast_lio2_core boundary. `MeasureGroup`,
// `Pose6D`, `set_pose6d()`, `G_m_s2`, `NUM_MATCH_POINTS` and the point type
// aliases are adapted from Fast-LIO2's common_lib.h
// (https://github.com/hku-mars/FAST_LIO, branch ROS2), GNU GPL v2.0-only,
// Copyright HKU-MARS lab, with the ROS types replaced by plain structs.
// See THIRD_PARTY_NOTICES.md at the repository root.
#pragma once

#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace fast_lio2_core
{
/// Standard gravity, m/s^2 (same constant Fast-LIO2 uses).
inline constexpr double G_m_s2 = 9.81;

/// Number of nearest map points used to fit a local plane for point-to-plane
/// matching (same as Fast-LIO2's `NUM_MATCH_POINTS`).
inline constexpr int NUM_MATCH_POINTS = 5;

/// Same point type Fast-LIO2 uses internally: XYZ + intensity, with the
/// `curvature` field repurposed to hold the per-point relative timestamp (in
/// milliseconds, relative to the scan start) needed for IMU-based deskewing.
/// Internal to fast_lio2_core only -- never crosses the public API boundary
/// (see `LidarPoint`/`XYZIPoint` below), since a `pcl::PointCloud<>`
/// allocated in one translation unit and freed in another -- built with a
/// differently-configured PCL/Eigen aligned-allocator ABI -- is exactly the
/// heap corruption this library works around with `-DPCL_NO_PRECOMPILE`
/// (see the CMakeLists.txt comment). Keeping all `pcl::PointCloud`
/// construction/destruction inside fast_lio2_core's own translation units
/// sidesteps the issue entirely rather than requiring every consumer to
/// replicate the same compile flags.
using PointType      = pcl::PointXYZINormal;
using PointCloudXYZI = pcl::PointCloud<PointType>;

/// One input LiDAR point, decoupled from any PCL type: the public-API
/// counterpart of `PointType` above.
struct LidarPoint
{
  float  x = 0, y = 0, z = 0;
  float  intensity   = 0;
  double t_offset_sec = 0;  //!< relative to the scan's start time
};
using LidarPointVector = std::vector<LidarPoint>;

/// One output (map/deskewed-scan) point: XYZI only, no per-point timing.
struct XYZIPoint
{
  float x = 0, y = 0, z = 0;
  float intensity = 0;
};
using XYZIPointVector = std::vector<XYZIPoint>;

/// A single IMU sample, decoupled from any ROS message type.
struct ImuSample
{
  double         t = 0;  //!< seconds, same clock as scan timestamps
  Eigen::Vector3d acc  = Eigen::Vector3d::Zero();  //!< m/s^2
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();  //!< rad/s
};

/// One LiDAR scan plus every IMU sample spanning its acquisition window,
/// as Fast-LIO2's `MeasureGroup` did, minus the ROS types.
struct MeasureGroup
{
  double               lidar_beg_time = 0;
  double               lidar_end_time = 0;
  PointCloudXYZI::Ptr  lidar;
  std::deque<ImuSample> imu;
};

/// Replacement for the ROS message `fast_lio::msg::Pose6D`: a cached
/// IMU pose sample used during backward-propagation deskewing.
struct Pose6D
{
  double offset_time = 0;
  double acc[3]      = {0, 0, 0};
  double gyr[3]       = {0, 0, 0};
  double vel[3]       = {0, 0, 0};
  double pos[3]       = {0, 0, 0};
  double rot[9]       = {1, 0, 0, 0, 1, 0, 0, 0, 1};
};

template <typename T>
Pose6D set_pose6d(
    const double t, const Eigen::Matrix<T, 3, 1>& a, const Eigen::Matrix<T, 3, 1>& g,
    const Eigen::Matrix<T, 3, 1>& v, const Eigen::Matrix<T, 3, 1>& p, const Eigen::Matrix<T, 3, 3>& R)
{
  Pose6D out;
  out.offset_time = t;
  for (int i = 0; i < 3; i++)
  {
    out.acc[i] = a(i);
    out.gyr[i] = g(i);
    out.vel[i] = v(i);
    out.pos[i] = p(i);
    for (int j = 0; j < 3; j++)
    {
      out.rot[i * 3 + j] = R(i, j);
    }
  }
  return out;
}

/// Algorithm parameters (mirrors the schema of Fast-LIO2's `config/*.yaml`).
struct Config
{
  /// LiDAR-to-IMU extrinsics: p_imu = extrinsic_R * p_lidar + extrinsic_T
  Eigen::Vector3d extrinsic_T = Eigen::Vector3d::Zero();
  Eigen::Matrix3d extrinsic_R = Eigen::Matrix3d::Identity();
  bool            extrinsic_est_en = false;

  double gyr_cov   = 0.1;
  double acc_cov   = 0.1;
  double b_gyr_cov = 1e-4;
  double b_acc_cov = 1e-4;

  double filter_size_surf   = 0.5;
  double filter_size_map    = 0.5;
  double cube_side_length   = 200.0;
  float  det_range          = 300.0f;
  int    max_iteration      = 4;

  /// Minimum point range [m]; closer returns are discarded (mirrors
  /// Fast-LIO2's `blind`). Needed on handheld/backpack rigs, where the
  /// carrier produces a dense near-field cluster that is rigidly attached
  /// to the sensor. Default 0 keeps every point, i.e. the previous
  /// behavior of this port.
  double blind = 0.0;

  /// Seconds since the first scan during which the estimate is not
  /// considered converged yet (mirrors Fast-LIO2's `INIT_TIME`).
  double init_time = 0.1;
};

/// Output of processing one LiDAR scan.
struct OdometryOutput
{
  double timestamp = 0;  //!< == the scan's end time

  /// world_from_imu, i.e. the pose of the IMU frame in the (arbitrary,
  /// gravity-unaligned) odometry world frame.
  Eigen::Isometry3d pose_imu = Eigen::Isometry3d::Identity();

  Eigen::Vector3d vel       = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_acc  = Eigen::Vector3d::Zero();
  Eigen::Vector3d gravity   = Eigen::Vector3d::Zero();

  /// Deskewed, downsampled scan in LiDAR body frame (at the scan end pose).
  std::shared_ptr<const XYZIPointVector> deskewed_points_body;
};

}  // namespace fast_lio2_core
