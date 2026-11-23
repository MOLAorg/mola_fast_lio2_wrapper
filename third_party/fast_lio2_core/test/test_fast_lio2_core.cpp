// SPDX-License-Identifier: BSD-3-Clause
// Copyright (C) 2026, Jose Luis Blanco-Claraco
// mola_fast_lio2_wrapper: original test code (not derived from Fast-LIO2).
#include <fast_lio2_core/FastLio2Core.h>

#include <gtest/gtest.h>

namespace
{
using fast_lio2_core::Config;
using fast_lio2_core::FastLio2Core;
using fast_lio2_core::G_m_s2;
using fast_lio2_core::ImuSample;
using fast_lio2_core::LidarPoint;
using fast_lio2_core::LidarPointVector;

// A flat 13x13 grid of points on a horizontal plane 1 m below the sensor,
// simulating a LiDAR looking down at a floor. `t_scan_period` gives each
// point a synthetic (linearly spaced) per-point relative timestamp.
LidarPointVector makeFloorScan(double t_scan_period)
{
  LidarPointVector cloud;
  int              idx = 0;
  for (double x = -3.0; x <= 3.0; x += 0.5)
  {
    for (double y = -3.0; y <= 3.0; y += 0.5)
    {
      LidarPoint p;
      p.x = static_cast<float>(x);
      p.y = static_cast<float>(y);
      // Tiny per-point jitter: a perfectly flat z (degenerate bounding box
      // along one axis) triggers a heap-buffer-overflow inside the system
      // libpcl_filters VoxelGrid on this machine; avoid it in the synthetic
      // fixture (a real scan never has exactly-equal z anyway).
      p.z            = -1.0f + 0.001f * static_cast<float>(idx % 7);
      p.intensity    = 0.5f;
      p.t_offset_sec = t_scan_period * (idx % 100) / 100.0;
      cloud.push_back(p);
      idx++;
    }
  }
  return cloud;
}

}  // namespace

TEST(FastLio2Core, StationaryRunConvergesAndBuildsMap)
{
  Config cfg;
  cfg.filter_size_surf  = 0.3;
  cfg.filter_size_map   = 0.3;
  cfg.cube_side_length  = 50.0;
  cfg.det_range         = 20.0f;
  cfg.max_iteration     = 3;
  cfg.gyr_cov = cfg.acc_cov = 0.01;
  cfg.b_gyr_cov = cfg.b_acc_cov = 1e-5;

  FastLio2Core core(cfg);

  constexpr double kImuRateHz  = 200.0;
  constexpr double kScanRateHz = 10.0;
  constexpr double kImuDt      = 1.0 / kImuRateHz;
  constexpr double kScanDt     = 1.0 / kScanRateHz;
  constexpr int    kNumScans   = 8;

  double t       = 0.0;
  double t_scan  = 0.0;
  bool   got_any_output = false;

  fast_lio2_core::OdometryOutput last_output;

  for (int scan_i = 0; scan_i < kNumScans; scan_i++)
  {
    const double t_scan_end = t_scan + kScanDt;

    // Feed stationary IMU samples spanning this scan's duration.
    while (t <= t_scan_end + kImuDt)
    {
      ImuSample s;
      s.t    = t;
      s.acc  = Eigen::Vector3d(0, 0, G_m_s2);
      s.gyro = Eigen::Vector3d::Zero();
      core.addImu(s);
      t += kImuDt;
    }

    core.addLidarScan(makeFloorScan(kScanDt), t_scan, t_scan_end);

    for (const auto& out : core.drainReady())
    {
      got_any_output = true;
      last_output    = out;
      ASSERT_TRUE(out.pose_imu.matrix().allFinite());
      ASSERT_TRUE(out.vel.allFinite());
    }

    t_scan = t_scan_end;
  }

  EXPECT_TRUE(got_any_output) << "expected at least one OdometryOutput after " << kNumScans << " scans";
  if (got_any_output)
  {
    // Purely stationary run: position should stay close to the origin.
    EXPECT_LT(last_output.pose_imu.translation().norm(), 2.0);
    // Gravity should be recovered pointing down, close to -G_m_s2 in Z.
    EXPECT_NEAR(last_output.gravity.z(), -G_m_s2, 2.0);
  }

  auto map = core.currentMap();
  ASSERT_TRUE(static_cast<bool>(map));
  EXPECT_GT(map->size(), 0u);
}
