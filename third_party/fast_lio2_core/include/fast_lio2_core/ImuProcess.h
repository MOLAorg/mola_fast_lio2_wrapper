// SPDX-License-Identifier: GPL-2.0-only
//
// Derived from Fast-LIO2's IMU_Processing.hpp
// (https://github.com/hku-mars/FAST_LIO, branch ROS2), GNU GPL v2.0-only,
// Copyright HKU-MARS lab. Ported off ROS message types and rclcpp::Time:
// the estimation logic (`IMU_init`, `UndistortPcl`) is otherwise unchanged.
// See THIRD_PARTY_NOTICES.md at the repository root.
#pragma once

#include <fast_lio2_core/Types.hpp>
#include <fast_lio2_core/use-ikfom.hpp>

#include <vector>

namespace fast_lio2_core
{
/// IMU forward-propagation and point-cloud deskew (backward propagation).
/// Port of Fast-LIO2's `ImuProcess` class.
class ImuProcess
{
 public:
  ImuProcess();

  void Reset();

  void set_extrinsic(const Eigen::Vector3d& transl, const Eigen::Matrix3d& rot);
  void set_gyr_cov(const Eigen::Vector3d& scaler);
  void set_acc_cov(const Eigen::Vector3d& scaler);
  void set_gyr_bias_cov(const Eigen::Vector3d& b_g);
  void set_acc_bias_cov(const Eigen::Vector3d& b_a);

  /// Runs IMU-bias/gravity initialization (first few calls) or IMU forward
  /// propagation + point-cloud deskew (afterwards). `pcl_out` receives the
  /// deskewed scan; it is left empty while still initializing.
  void Process(
      const MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state,
      PointCloudXYZI::Ptr pcl_out);

  double first_lidar_time = 0;

  bool needsMoreInit() const { return imu_need_init_; }

 private:
  void IMU_init(const MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, int& N);
  void UndistortPcl(
      const MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, PointCloudXYZI& pcl_out);

  Eigen::Matrix<double, 12, 12> Q_;

  Eigen::Vector3d cov_acc_       = Eigen::Vector3d(0.1, 0.1, 0.1);
  Eigen::Vector3d cov_gyr_       = Eigen::Vector3d(0.1, 0.1, 0.1);
  Eigen::Vector3d cov_acc_scale_ = Eigen::Vector3d(1, 1, 1);
  Eigen::Vector3d cov_gyr_scale_ = Eigen::Vector3d(1, 1, 1);
  Eigen::Vector3d cov_bias_gyr_  = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
  Eigen::Vector3d cov_bias_acc_  = Eigen::Vector3d(0.0001, 0.0001, 0.0001);

  ImuSample last_imu_;

  std::vector<Pose6D> imu_pose_;

  Eigen::Matrix3d lidar_R_wrt_imu_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d lidar_T_wrt_imu_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d mean_acc_    = Eigen::Vector3d(0, 0, -1.0);
  Eigen::Vector3d mean_gyr_    = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d angvel_last_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc_s_last_  = Eigen::Vector3d::Zero();

  double last_lidar_end_time_ = 0;
  int    init_iter_num_       = 1;
  bool   b_first_frame_       = true;
  bool   imu_need_init_       = true;
};

}  // namespace fast_lio2_core
