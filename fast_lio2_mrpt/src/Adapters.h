// SPDX-License-Identifier: BSD-3-Clause
// Copyright (C) 2026, Jose Luis Blanco-Claraco
// See LICENSES/BSD-3-Clause.txt and THIRD_PARTY_NOTICES.md.

/**
 * @file   Adapters.h
 * @brief  Conversions between MRPT and fast_lio2_core plain types
 *         (private to this library, not installed).
 */
#pragma once

#include <fast_lio2_core/Types.hpp>

// clang-format off
// fast_lio2_core/Types.hpp (above) already includes enough PCL headers to
// define PCL_LINEAR_VERSION, which makes mrpt/maps/CPointsMap.h (below)
// conditionally compile inline PCL PCD I/O methods -- but only this header
// actually declares the `pcl::io::savePCDFile`/`loadPCDFile` symbols those
// methods call. MRPT's own comment on those methods: "requires user code to
// include PCL before MRPT headers". Included here (not in the .cpp) so
// every translation unit that includes this header gets it consistently.
#include <pcl/io/pcd_io.h>
// clang-format on

#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>

namespace fast_lio2_mrpt
{
/** See the .cpp file for the full documentation of the conversion logic. */
fast_lio2_core::LidarPointVector toFastLio2Cloud(
  const mrpt::obs::CObservationPointCloud & obs, double fallback_scan_period, double & t_beg,
  double & t_end, double blind = 0.0, bool ignorePointTimestamps = false);

fast_lio2_core::ImuSample toFastLio2Imu(const mrpt::obs::CObservationIMU & obs);

}  // namespace fast_lio2_mrpt
