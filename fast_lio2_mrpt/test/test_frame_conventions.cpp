// SPDX-License-Identifier: BSD-3-Clause
// Copyright (C) 2026, Jose Luis Blanco-Claraco
// See LICENSES/BSD-3-Clause.txt and THIRD_PARTY_NOTICES.md.

/** Frame conventions of the pose this wrapper reports.
 *
 * The filter estimates the IMU in the world frame; the wrapper reports
 * base_link. Both extrinsics are stated in the "wrt" direction and both have
 * to be inverted on the way. Composing either of them uninverted is
 * self-consistent for an extrinsic that is its own inverse -- identity above
 * all -- which is exactly what lets such a mistake ship unnoticed, so every
 * case below uses a non-involutive extrinsic on purpose.
 */

#include <fast_lio2_mrpt/Odometry.h>
#include <gtest/gtest.h>
#include <mrpt/poses/CPose3D.h>

#include <cmath>

using fast_lio2_mrpt::Odometry;
using mrpt::poses::CPose3D;

namespace
{

// A deliberately asymmetric IMU-to-LiDAR extrinsic: T_lidar_imu^-1 differs
// from T_lidar_imu in both its rotation and its translation.
CPose3D imuInLidar()
{
  return CPose3D::FromXYZYawPitchRoll(
    0.018771, -0.008218, -0.070474, mrpt::DEG2RAD(-90.6263), mrpt::DEG2RAD(-0.1665),
    mrpt::DEG2RAD(-0.1287));
}

// Oxford Spires: base_link is a 180 deg yaw and 0.124 m from the LiDAR.
CPose3D lidarInBaseLink()
{
  return CPose3D::FromXYZYawPitchRoll(0, 0, 0.124, mrpt::DEG2RAD(180.0), 0, 0);
}

void expectPoseNear(const CPose3D & a, const CPose3D & b, double tol)
{
  for (int i = 0; i < 6; i++) {
    EXPECT_NEAR(a.asTPose()[i], b.asTPose()[i], tol)
      << "component " << i << ": " << a.asString() << " vs " << b.asString();
  }
}

}  // namespace

TEST(FrameConventions, IdentityExtrinsicsAreANoOp)
{
  const auto worldFromImu = CPose3D::FromXYZYawPitchRoll(
    1.0, 2.0, 3.0, mrpt::DEG2RAD(30.0), mrpt::DEG2RAD(10.0), mrpt::DEG2RAD(-5.0));
  expectPoseNear(
    Odometry::worldFromBaseLink(worldFromImu, CPose3D::Identity(), CPose3D::Identity()),
    worldFromImu, 1e-9);
}

TEST(FrameConventions, LidarOutputInvertsTheImuExtrinsic)
{
  // With base_link left at identity the reported pose is the LiDAR's, which
  // is T_w_imu . T_imu_lidar -- NOT T_w_imu . T_lidar_imu. The two differ by
  // T_lidar_imu squared, so this is the case that catches the sign of the
  // inversion rather than merely its presence.
  const auto worldFromImu = CPose3D::FromXYZYawPitchRoll(
    1.0, 2.0, 3.0, mrpt::DEG2RAD(30.0), mrpt::DEG2RAD(10.0), mrpt::DEG2RAD(-5.0));
  CPose3D lidarInImu = imuInLidar();
  lidarInImu.inverse();

  const auto got = Odometry::worldFromBaseLink(worldFromImu, imuInLidar(), CPose3D::Identity());
  expectPoseNear(got, worldFromImu + lidarInImu, 1e-9);

  const auto wrong = worldFromImu + imuInLidar();
  EXPECT_GT((got.asTPose() - wrong.asTPose()).norm(), 0.1)
    << "the uninverted composition must not coincide with the right one, "
       "or this test proves nothing";
}

TEST(FrameConventions, RoundTripsThroughBothExtrinsics)
{
  // Put base_link somewhere known, derive where the IMU must be, and check
  // that the reported pose lands back on base_link.
  const auto worldFromBase = CPose3D::FromXYZYawPitchRoll(
    10.0, -4.0, 0.5, mrpt::DEG2RAD(-70.0), mrpt::DEG2RAD(3.0), mrpt::DEG2RAD(2.0));
  const auto worldFromImu = worldFromBase + lidarInBaseLink() + imuInLidar();

  expectPoseNear(
    Odometry::worldFromBaseLink(worldFromImu, imuInLidar(), lidarInBaseLink()), worldFromBase,
    1e-9);
}

TEST(FrameConventions, ComposesAsAConstantRightMultiplication)
{
  // What the module actually does: build the composition once against an
  // identity world pose, then right-compose every estimate with it.
  const auto constant =
    Odometry::worldFromBaseLink(CPose3D::Identity(), imuInLidar(), lidarInBaseLink());

  for (double yawDeg : {0.0, 45.0, 137.0, -179.0}) {
    const auto worldFromImu = CPose3D::FromXYZYawPitchRoll(
      yawDeg * 0.01, 1.0, -2.0, mrpt::DEG2RAD(yawDeg), mrpt::DEG2RAD(4.0), mrpt::DEG2RAD(-6.0));
    expectPoseNear(
      Odometry::worldFromBaseLink(worldFromImu, imuInLidar(), lidarInBaseLink()),
      worldFromImu + constant, 1e-9);
  }
}
