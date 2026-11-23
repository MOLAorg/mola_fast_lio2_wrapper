// SPDX-License-Identifier: BSD-3-Clause
// Copyright (C) 2026, Jose Luis Blanco-Claraco
// See LICENSES/BSD-3-Clause.txt and THIRD_PARTY_NOTICES.md.
//
// The program built from this file links fast_lio2_core, which is
// GPL-2.0-only, so the program as a whole is distributed under GPL-2.0-only.
// It must not link GPL-3.0 code (MOLA libraries included).

/**
 * @file   mola-fast-lio2-cli.cpp
 * @brief  Offline Fast-LIO2 over an MRPT rawlog, without dropping any scan.
 *
 * Reads the rawlog one entry at a time, feeds it to fast_lio2_mrpt::Odometry
 * (which processes it to completion before returning) and saves the
 * estimated trajectory in TUM format. Datasets in other formats can be
 * converted to a rawlog first, or streamed through standard input.
 */

#include <fast_lio2_mrpt/Odometry.h>
#include <fast_lio2_mrpt/yaml_env.h>
#include <mrpt/core/Clock.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CCompressedInputStream.h>
#include <mrpt/io/lazy_load_path.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/datetime.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/typemeta/TEnumType.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

namespace
{
struct Options
{
  std::string configFile;
  std::string inputRawlog;
  std::string outputTum;
  std::string verbosity = "INFO";
  size_t maxEntries = 0;
  bool syntheticImu = false;
};

/// An uncompressed rawlog read from the standard input.
class StandardInputStream : public mrpt::io::CStream
{
public:
  size_t Read(void * buffer, size_t count) override
  {
    const size_t n = std::fread(buffer, 1, count, stdin);
    position_ += n;
    return n;
  }
  size_t Write(const void * /*buffer*/, size_t /*count*/) override
  {
    THROW_EXCEPTION("The standard input is read-only");
  }
  uint64_t Seek(int64_t /*offset*/, TSeekOrigin /*origin*/) override
  {
    THROW_EXCEPTION("The standard input cannot seek");
  }
  uint64_t getTotalBytesCount() const override { return 0; }
  uint64_t getPosition() const override { return position_; }

private:
  uint64_t position_ = 0;
};

std::unique_ptr<mrpt::io::CStream> openRawlog(const std::string & path)
{
  if (path == "-") {
    return std::make_unique<StandardInputStream>();
  }
  // Observations stored outside the rawlog file, if any, live next to it.
  const std::string externalDir = mrpt::obs::CRawlog::detectImagesDirectory(path);
  if (mrpt::system::directoryExists(externalDir)) {
    mrpt::io::setLazyLoadPathBase(externalDir);
  }
  // gzip, zstd or uncompressed:
  return std::make_unique<mrpt::io::CCompressedInputStream>(path);
}

/// The observation used from one rawlog entry: its first point cloud, or
/// else its first IMU reading. At most one observation is used per entry.
mrpt::obs::CObservation::ConstPtr pickObservation(
  const mrpt::serialization::CSerializable::Ptr & entry, bool & isScan)
{
  isScan = false;
  auto sf = std::dynamic_pointer_cast<mrpt::obs::CSensoryFrame>(entry);
  if (!sf) {
    sf = mrpt::obs::CSensoryFrame::Create();
    if (auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(entry); obs) {
      sf->insert(obs);
    } else if (!std::dynamic_pointer_cast<mrpt::obs::CActionCollection>(entry)) {
      THROW_EXCEPTION_FMT("Unexpected class in rawlog: '%s'", entry->GetRuntimeClass()->className);
    }
  }
  if (auto scan = sf->getObservationByClass<mrpt::obs::CObservationPointCloud>(); scan) {
    isScan = true;
    return scan;
  }
  return sf->getObservationByClass<mrpt::obs::CObservationIMU>();
}

/// A stationary, level IMU reading 1 ms before a scan, for LiDAR-only
/// datasets: Fast-LIO2 needs IMU data to propagate the filter, and with this
/// the motion estimate comes from the LiDAR alone.
mrpt::obs::CObservationIMU::Ptr syntheticImuBefore(const mrpt::obs::CObservation & scan)
{
  constexpr double kStandardGravity = 9.80665;  // m/s^2, +g on the up axis

  const double tScan = mrpt::Clock::toDouble(scan.timestamp);
  auto imu = mrpt::obs::CObservationIMU::Create();
  imu->sensorLabel = "imu";
  // Clamped: a sequence-relative clock may start at exactly zero.
  imu->timestamp = mrpt::Clock::fromDouble(std::max(0.0, tScan - 0.001));
  imu->set(mrpt::obs::IMU_X_ACC, 0.0);
  imu->set(mrpt::obs::IMU_Y_ACC, 0.0);
  imu->set(mrpt::obs::IMU_Z_ACC, kStandardGravity);
  imu->set(mrpt::obs::IMU_WX, 0.0);
  imu->set(mrpt::obs::IMU_WY, 0.0);
  imu->set(mrpt::obs::IMU_WZ, 0.0);
  return imu;
}

void printProgress(
  const mrpt::io::CStream & in, size_t entries, size_t scans, double tStart, bool last)
{
  const double elapsed = mrpt::Clock::nowDouble() - tStart;
  const uint64_t total = in.getTotalBytesCount();
  std::string eta = "?";
  double percent = 0;
  if (total > 0) {
    const double fraction = static_cast<double>(in.getPosition()) / static_cast<double>(total);
    percent = 100.0 * fraction;
    if (fraction > 0) {
      eta = mrpt::system::formatTimeInterval(elapsed * (1.0 / fraction - 1.0));
    }
  }
  std::printf(
    "\r[%5.1f%%] entries: %zu, scans: %zu, elapsed: %s ETA=%s%s", percent, entries, scans,
    mrpt::system::formatTimeInterval(elapsed).c_str(), eta.c_str(), last ? "\n" : "");
  std::fflush(stdout);
}

int run(const Options & opt)
{
  fast_lio2_mrpt::Odometry odometry;
  odometry.setMinLoggingLevel(
    mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>::name2value(opt.verbosity));
  odometry.initialize(fast_lio2_mrpt::loadYamlFile(opt.configFile));

  const auto in = openRawlog(opt.inputRawlog);
  auto archive = mrpt::serialization::archiveFrom(*in);

  size_t entries = 0;
  size_t scans = 0;
  const double tStart = mrpt::Clock::nowDouble();
  double tLastPrint = tStart;

  while (opt.maxEntries == 0 || entries < opt.maxEntries) {
    mrpt::serialization::CSerializable::Ptr entry;
    try {
      entry = archive.ReadObject();
    } catch (const mrpt::serialization::CExceptionEOF &) {
      break;
    }
    entries++;

    bool isScan = false;
    const auto obs = pickObservation(entry, isScan);
    if (!obs) {
      continue;
    }
    // Rawlogs may keep observation data in external files: read it now.
    // No effect otherwise.
    obs->load();
    if (isScan) {
      scans++;
      if (opt.syntheticImu) {
        odometry.processObservation(syntheticImuBefore(*obs));
      }
    }
    odometry.processObservation(obs);

    if (const double tNow = mrpt::Clock::nowDouble(); tNow - tLastPrint > 1.0) {
      tLastPrint = tNow;
      printProgress(*in, entries, scans, tStart, false);
    }
  }
  printProgress(*in, entries, scans, tStart, true);

  std::cout << "Done. Rawlog entries read: " << entries << ", LiDAR scans: " << scans << "\n";

  if (!opt.outputTum.empty()) {
    std::cout << "Saving the estimated trajectory in TUM format to: " << opt.outputTum << "\n";
    odometry.trajectory().saveToTextFile_TUM(opt.outputTum);
  }
  if (const auto file = odometry.trajectoryOutputFile(); file) {
    std::cout << "Saving the estimated trajectory in TUM format to: " << *file << "\n";
    odometry.trajectory().saveToTextFile_TUM(*file);
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  CLI::App app{
    "Runs Fast-LIO2 offline over an MRPT rawlog, without dropping any scan, and saves the "
    "estimated trajectory."};
  app.add_option("-c,--config", opt.configFile, "Fast-LIO2 pipeline YAML file")->required();
  app
    .add_option(
      "--input-rawlog", opt.inputRawlog,
      "Input dataset in MRPT rawlog format (gzip, zstd or uncompressed), or '-' to read an "
      "uncompressed rawlog from the standard input")
    ->required();
  app.add_option(
    "--output-tum-path", opt.outputTum, "Save the estimated trajectory to this file (TUM format)");
  app.add_option("--only-first-n", opt.maxEntries, "Process only the first N rawlog entries");
  app.add_flag(
    "--synthetic-imu", opt.syntheticImu,
    "Insert a stationary, level IMU reading (label 'imu') before each LiDAR scan, for "
    "LiDAR-only datasets");
  app.add_option("-v,--verbosity", opt.verbosity, "Verbosity level: ERROR|WARN|INFO|DEBUG");

  CLI11_PARSE(app, argc, argv);

  try {
    return run(opt);
  } catch (const std::exception & e) {
    std::cerr << "Error: " << mrpt::exception_to_str(e) << "\n";
    return 1;
  }
}
