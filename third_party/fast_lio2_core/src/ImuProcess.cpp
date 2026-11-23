// SPDX-License-Identifier: GPL-2.0-only
//
// Derived from Fast-LIO2's IMU_Processing.hpp
// (https://github.com/hku-mars/FAST_LIO, branch ROS2), GNU GPL v2.0-only,
// Copyright HKU-MARS lab. Ported off ROS message types and rclcpp::Time:
// the estimation logic (`IMU_init`, `UndistortPcl`) is otherwise unchanged.
// See THIRD_PARTY_NOTICES.md at the repository root.
#include <fast_lio2_core/ImuProcess.h>
#include <fast_lio2_core/so3_math.h>

#include <algorithm>

namespace fast_lio2_core
{

namespace
{
constexpr int MAX_INI_COUNT = 10;

bool time_list(const PointType& x, const PointType& y) { return x.curvature < y.curvature; }

Eigen::Matrix<double, 12, 12> process_noise_cov_identity()
{
  return Eigen::Matrix<double, 12, 12>::Identity();
}
}  // namespace

ImuProcess::ImuProcess()
{
  Q_ = process_noise_cov_identity();
  Q_.block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity() * 0.0001;
  Q_.block<3, 3>(3, 3)  = Eigen::Matrix3d::Identity() * 0.0001;
  Q_.block<3, 3>(6, 6)  = Eigen::Matrix3d::Identity() * 0.00001;
  Q_.block<3, 3>(9, 9)  = Eigen::Matrix3d::Identity() * 0.00001;
}

void ImuProcess::Reset()
{
  mean_acc_         = Eigen::Vector3d(0, 0, -1.0);
  mean_gyr_         = Eigen::Vector3d(0, 0, 0);
  angvel_last_      = Eigen::Vector3d::Zero();
  imu_need_init_    = true;
  init_iter_num_    = 1;
  imu_pose_.clear();
  last_imu_ = ImuSample();
}

void ImuProcess::set_extrinsic(const Eigen::Vector3d& transl, const Eigen::Matrix3d& rot)
{
  lidar_T_wrt_imu_ = transl;
  lidar_R_wrt_imu_ = rot;
}

void ImuProcess::set_gyr_cov(const Eigen::Vector3d& scaler) { cov_gyr_scale_ = scaler; }
void ImuProcess::set_acc_cov(const Eigen::Vector3d& scaler) { cov_acc_scale_ = scaler; }
void ImuProcess::set_gyr_bias_cov(const Eigen::Vector3d& b_g) { cov_bias_gyr_ = b_g; }
void ImuProcess::set_acc_bias_cov(const Eigen::Vector3d& b_a) { cov_bias_acc_ = b_a; }

void ImuProcess::IMU_init(const MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, int& N)
{
  // 1. initialize gravity, gyro bias, acc/gyro covariance from a stationary
  //    lead-in of IMU-only samples.
  // 2. normalize the acceleration measurements to unit gravity.
  Eigen::Vector3d cur_acc, cur_gyr;

  if (b_first_frame_)
  {
    Reset();
    N              = 1;
    b_first_frame_ = false;
    mean_acc_       = meas.imu.front().acc;
    mean_gyr_       = meas.imu.front().gyro;
    first_lidar_time = meas.lidar_beg_time;
  }

  for (const auto& imu : meas.imu)
  {
    cur_acc = imu.acc;
    cur_gyr = imu.gyro;

    mean_acc_ += (cur_acc - mean_acc_) / N;
    mean_gyr_ += (cur_gyr - mean_gyr_) / N;

    cov_acc_ = cov_acc_ * (N - 1.0) / N + (cur_acc - mean_acc_).cwiseProduct(cur_acc - mean_acc_) * (N - 1.0) / (N * N);
    cov_gyr_ = cov_gyr_ * (N - 1.0) / N + (cur_gyr - mean_gyr_).cwiseProduct(cur_gyr - mean_gyr_) * (N - 1.0) / (N * N);

    N++;
  }

  state_ikfom init_state = kf_state.get_x();
  init_state.grav        = S2(-mean_acc_ / mean_acc_.norm() * G_m_s2);
  init_state.bg           = mean_gyr_;
  init_state.offset_T_L_I = lidar_T_wrt_imu_;
  init_state.offset_R_L_I = lidar_R_wrt_imu_;
  kf_state.change_x(init_state);

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();
  init_P.setIdentity();
  init_P(6, 6) = init_P(7, 7) = init_P(8, 8) = 0.00001;
  init_P(9, 9) = init_P(10, 10) = init_P(11, 11) = 0.00001;
  init_P(15, 15) = init_P(16, 16) = init_P(17, 17) = 0.0001;
  init_P(18, 18) = init_P(19, 19) = init_P(20, 20) = 0.001;
  init_P(21, 21) = init_P(22, 22) = 0.00001;
  kf_state.change_P(init_P);

  last_imu_ = meas.imu.back();
}

void ImuProcess::UndistortPcl(
    const MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, PointCloudXYZI& pcl_out)
{
  // Prepend the last frame's tail IMU sample to this frame's head.
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double& pcl_beg_time = meas.lidar_beg_time;
  const double& pcl_end_time = meas.lidar_end_time;

  // Sort points by their per-point relative time offset.
  pcl_out = *(meas.lidar);
  std::sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);

  state_ikfom imu_state = kf_state.get_x();
  imu_pose_.clear();
  imu_pose_.push_back(set_pose6d(
      0.0, acc_s_last_, angvel_last_, Eigen::Vector3d(imu_state.vel), Eigen::Vector3d(imu_state.pos),
      imu_state.rot.toRotationMatrix()));

  // Forward propagation at each IMU sample.
  Eigen::Vector3d angvel_avr, acc_avr;
  double          dt = 0;
  input_ikfom     in;

  for (auto it_imu = v_imu.begin(); it_imu < v_imu.end() - 1; it_imu++)
  {
    const ImuSample& head = *it_imu;
    const ImuSample& tail = *(it_imu + 1);

    if (tail.t < last_lidar_end_time_) continue;

    angvel_avr = 0.5 * (head.gyro + tail.gyro);
    acc_avr    = 0.5 * (head.acc + tail.acc);

    acc_avr = acc_avr * G_m_s2 / mean_acc_.norm();

    if (head.t < last_lidar_end_time_)
    {
      dt = tail.t - last_lidar_end_time_;
    }
    else
    {
      dt = tail.t - head.t;
    }

    in.acc  = acc_avr;
    in.gyro = angvel_avr;
    Q_.block<3, 3>(0, 0).diagonal() = cov_gyr_;
    Q_.block<3, 3>(3, 3).diagonal() = cov_acc_;
    Q_.block<3, 3>(6, 6).diagonal() = cov_bias_gyr_;
    Q_.block<3, 3>(9, 9).diagonal() = cov_bias_acc_;
    kf_state.predict(dt, Q_, in);

    imu_state    = kf_state.get_x();
    angvel_last_ = angvel_avr - imu_state.bg;
    acc_s_last_  = imu_state.rot * (acc_avr - imu_state.ba);
    for (int i = 0; i < 3; i++)
    {
      acc_s_last_[i] += imu_state.grav[i];
    }
    const double offs_t = tail.t - pcl_beg_time;
    imu_pose_.push_back(set_pose6d(
        offs_t, acc_s_last_, angvel_last_, Eigen::Vector3d(imu_state.vel), Eigen::Vector3d(imu_state.pos),
        imu_state.rot.toRotationMatrix()));
  }

  // Predict up to the scan's end time.
  const double imu_end_time = v_imu.back().t;
  const double note         = pcl_end_time > imu_end_time ? 1.0 : -1.0;
  dt                        = note * (pcl_end_time - imu_end_time);
  kf_state.predict(dt, Q_, in);

  imu_state             = kf_state.get_x();
  last_imu_              = meas.imu.back();
  last_lidar_end_time_   = pcl_end_time;

  // Backward propagation: deskew each point to the scan-end pose.
  if (pcl_out.points.empty()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = imu_pose_.end() - 1; it_kp != imu_pose_.begin(); it_kp--)
  {
    const Pose6D& head = *(it_kp - 1);
    const Pose6D& tail = *it_kp;

    Eigen::Matrix3d R_imu;
    R_imu << head.rot[0], head.rot[1], head.rot[2], head.rot[3], head.rot[4], head.rot[5], head.rot[6], head.rot[7],
        head.rot[8];
    Eigen::Vector3d vel_imu(head.vel[0], head.vel[1], head.vel[2]);
    Eigen::Vector3d pos_imu(head.pos[0], head.pos[1], head.pos[2]);
    Eigen::Vector3d acc_imu(tail.acc[0], tail.acc[1], tail.acc[2]);
    angvel_avr = Eigen::Vector3d(tail.gyr[0], tail.gyr[1], tail.gyr[2]);

    for (; it_pcl->curvature / double(1000) > head.offset_time; it_pcl--)
    {
      dt = it_pcl->curvature / double(1000) - head.offset_time;

      // Transform to the scan-end frame using only the rotation.
      // Compensation direction is the inverse of the frame's motion
      // direction: to compensate a point at timestamp-i to the frame-e,
      // P_compensate = R_imu_e^T * (R_i * P_i + T_ei), T_ei in global frame.
      Eigen::Matrix3d R_i(R_imu * Exp(angvel_avr, dt));

      Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - Eigen::Vector3d(imu_state.pos));
      Eigen::Vector3d P_compensate =
          imu_state.offset_R_L_I.conjugate() *
          (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) -
           imu_state.offset_T_L_I);

      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;
    }
  }
}

void ImuProcess::Process(
    const MeasureGroup& meas, esekfom::esekf<state_ikfom, 12, input_ikfom>& kf_state, PointCloudXYZI::Ptr pcl_out)
{
  if (meas.imu.empty()) return;
  if (!meas.lidar) return;

  if (imu_need_init_)
  {
    IMU_init(meas, kf_state, init_iter_num_);

    last_imu_ = meas.imu.back();

    if (init_iter_num_ > MAX_INI_COUNT)
    {
      cov_acc_ *= std::pow(G_m_s2 / mean_acc_.norm(), 2);
      imu_need_init_ = false;

      cov_acc_ = cov_acc_scale_;
      cov_gyr_ = cov_gyr_scale_;
    }
    return;
  }

  UndistortPcl(meas, kf_state, *pcl_out);
}

}  // namespace fast_lio2_core
