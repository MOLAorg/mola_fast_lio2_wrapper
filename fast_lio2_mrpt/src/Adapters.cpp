// SPDX-License-Identifier: BSD-3-Clause
// Copyright (C) 2026, Jose Luis Blanco-Claraco
// See LICENSES/BSD-3-Clause.txt and THIRD_PARTY_NOTICES.md.

/**
 * @file   Adapters.cpp
 * @brief  Conversions between MRPT and fast_lio2_core plain types
 */
#include "Adapters.h"

#include <mrpt/maps/CPointsMap.h>

#include <algorithm>
#include <cmath>

namespace fast_lio2_mrpt
{
using fast_lio2_core::LidarPoint;
using fast_lio2_core::LidarPointVector;

/** Converts an incoming LiDAR observation into Fast-LIO2 points, and
 * computes the scan's absolute [t_beg, t_end] time span.
 *
 * If the point cloud carries a per-point relative-time channel
 * (`CPointsMap::POINT_FIELD_TIMESTAMP`, seconds, any convention -- centered
 * on the observation timestamp, start-relative, etc.), it is used directly:
 * each point's `t_offset_sec` (relative to scan start) is computed by
 * re-normalizing relative to this scan's own min/max, so no assumption
 * about the specific dataset's time convention is needed.
 *
 * Otherwise (no per-point timestamps, e.g. a point cloud converted from a
 * plain `PointCloud2` with no `t`/`time` field), falls back to Fast-LIO2's
 * own strategy for such LiDARs: synthesize an offset from each point's
 * azimuth angle, assuming constant angular velocity across
 * `fallback_scan_period` seconds and that the observation's timestamp marks
 * the scan's end (last, forward-facing point).
 */
LidarPointVector toFastLio2Cloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end, double blind, bool ignorePointTimestamps)
{
  LidarPointVector out;
  const double blind2 = blind * blind;

  const auto & pts = obs.pointcloud;
  if (!pts) {
    t_beg = t_end = mrpt::Clock::toDouble(obs.timestamp);
    return out;
  }

  const auto & xs = pts->getPointsBufferRef_x();
  const auto & ys = pts->getPointsBufferRef_y();
  const auto & zs = pts->getPointsBufferRef_z();
  const auto n = xs.size();

  const auto * is =
    pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_INTENSITY);
  const auto * ts =
    pts->getPointsBufferRef_float_field(mrpt::maps::CPointsMap::POINT_FIELD_TIMESTAMP);

  out.resize(n);

  const double obsTime = mrpt::Clock::toDouble(obs.timestamp);

  if (ignorePointTimestamps) {
    // Already motion-compensated input: every point at the observation time.
    t_beg = t_end = obsTime;
    size_t k = 0;
    for (size_t i = 0; i < n; i++) {
      const double r2 = static_cast<double>(xs[i]) * xs[i] + static_cast<double>(ys[i]) * ys[i] +
                        static_cast<double>(zs[i]) * zs[i];
      if (r2 < blind2) {
        continue;
      }
      LidarPoint & p = out[k++];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;
      p.t_offset_sec = 0.0;
    }
    out.resize(k);
    return out;
  }

  if (ts && ts->size() == n && n > 0) {
    float tMin = (*ts)[0], tMax = (*ts)[0];
    for (size_t i = 0; i < n; i++) {
      tMin = std::min(tMin, (*ts)[i]);
      tMax = std::max(tMax, (*ts)[i]);
    }
    t_beg = obsTime + tMin;
    t_end = obsTime + tMax;

    size_t k = 0;
    for (size_t i = 0; i < n; i++) {
      const double r2 = static_cast<double>(xs[i]) * xs[i] + static_cast<double>(ys[i]) * ys[i] +
                        static_cast<double>(zs[i]) * zs[i];
      if (r2 < blind2) {
        continue;
      }
      LidarPoint & p = out[k++];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;
      p.t_offset_sec = static_cast<double>((*ts)[i] - tMin);
    }
    out.resize(k);
  } else {
    // Fallback: assume `obsTime` is the scan's end (forward-facing point),
    // and synthesize each point's offset from its azimuth angle.
    t_beg = obsTime - fallback_scan_period;
    t_end = obsTime;

    size_t k = 0;
    for (size_t i = 0; i < n; i++) {
      const double r2 = static_cast<double>(xs[i]) * xs[i] + static_cast<double>(ys[i]) * ys[i] +
                        static_cast<double>(zs[i]) * zs[i];
      if (r2 < blind2) {
        continue;
      }
      LidarPoint & p = out[k++];
      p.x = xs[i];
      p.y = ys[i];
      p.z = zs[i];
      p.intensity = is && i < is->size() ? (*is)[i] : 0.f;

      // Azimuth in [0, 2*pi), 0 at the scan start (fallback_scan_period ago).
      const double azimuth = std::atan2(static_cast<double>(ys[i]), static_cast<double>(xs[i]));
      const double azimuth01 = (azimuth + M_PI) / (2.0 * M_PI);
      p.t_offset_sec = azimuth01 * fallback_scan_period;
    }
    out.resize(k);
  }

  return out;
}

fast_lio2_core::ImuSample toFastLio2Imu(const mrpt::obs::CObservationIMU & obs)
{
  fast_lio2_core::ImuSample s;
  s.t = mrpt::Clock::toDouble(obs.timestamp);
  s.acc = Eigen::Vector3d(
    obs.get(mrpt::obs::IMU_X_ACC), obs.get(mrpt::obs::IMU_Y_ACC), obs.get(mrpt::obs::IMU_Z_ACC));
  s.gyro = Eigen::Vector3d(
    obs.get(mrpt::obs::IMU_WX), obs.get(mrpt::obs::IMU_WY), obs.get(mrpt::obs::IMU_WZ));
  return s;
}

}  // namespace fast_lio2_mrpt
