/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2026, Jose Luis Blanco-Claraco
 SPDX-License-Identifier: GPL-3.0-only
 See LICENSES/GPL-3.0-only.txt and THIRD_PARTY_NOTICES.md.
*/

/**
 * @file   mola-fast-lio2-dataset2rawlog.cpp
 * @brief  Writes a MOLA-supported dataset as an MRPT rawlog, to a file or to
 *         the standard output.
 *
 * This program links MOLA (GPL-3.0) and contains no Fast-LIO2 code. It is a
 * separate program from mola-fast-lio2-cli (GPL-2.0-only): the two only
 * exchange an MRPT rawlog, through a file or a pipe, for example:
 *
 *   mola-fast-lio2-dataset2rawlog --input-kitti-seq 00 -o - | \
 *     mola-fast-lio2-cli -c fastlio2-kitti.yaml --synthetic-imu \
 *       --input-rawlog - --output-tum-path kitti00.tum
 *
 * The dataset source helpers are adapted from mola_lidar_odometry's
 * mola-lidar-odometry-cli.cpp, so the same environment variables work here.
 */

#include <mola_kernel/interfaces/OfflineDatasetSource.h>
#include <mola_kernel/pretty_print_exception.h>
#include <mola_yaml/yaml_helpers.h>
#include <mrpt/core/exceptions.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/obs/CObservationPointCloud.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/typemeta/TEnumType.h>
#include <unistd.h>

#include <CLI/CLI.hpp>
#include <algorithm>
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#if defined(HAVE_MOLA_INPUT_MULRAN)
#include <mola_input_mulran_dataset/MulranDataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
#include <mola_input_rosbag2/Rosbag2Dataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
#include <mola_input_rosbag1/Rosbag1Dataset.h>
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
#include <mola_input_kitti_dataset/KittiOdometryDataset.h>
#endif

namespace
{
struct Options
{
  std::string output;
  std::string outputGt;
  std::string verbosity = "INFO";
  size_t maxEntries = 0;
  std::string mulranSeq;
  std::string kittiSeq;
  std::string rosbag2;
  std::string rosbag1;
  // Shared by both bag formats.
  std::string arg_lidarTopic{"/lidar"};
  std::string arg_imuTopic{"/imu"};
};

#if defined(HAVE_MOLA_INPUT_MULRAN)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_mulran(
  const std::string & mulranSequence, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::MulranDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      base_dir: ${MULRAN_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      publish_lidar: true
      publish_ground_truth: true
)"""",
    mulranSequence.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG2)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag2(
  const Options & cli, const std::string & rosbag2file, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::Rosbag2Dataset>();
  o->setMinLoggingLevel(logLevel);

  // A comma-separated value becomes a YAML sequence, so that a sequence split
  // across several bag directories (e.g. Oxford Spires keble-college-04, whose
  // two parts share a base timestamp and are two halves of one continuous
  // recording) can be replayed as the single sequence it is. Rosbag2Dataset
  // accepts either a scalar or a sequence for 'rosbag_filename'.
  std::string bagsYaml;
  {
    std::vector<std::string> parts;
    mrpt::system::tokenize(rosbag2file, ",", parts);
    ASSERT_(!parts.empty());
    if (parts.size() == 1) {
      bagsYaml = "'" + mrpt::system::trim(parts[0]) + "'";
    } else {
      for (const auto & p : parts) {
        bagsYaml += "\n        - '" + mrpt::system::trim(p) + "'";
      }
    }
  }

  // Fixed sensor poses (env vars), for bags with no /tf or /tf_static (e.g.
  // Oxford Spires): same env var names as mola-lidar-odometry-cli's own
  // dataset_from_rosbag2(), so the same override snippet works for both.
  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rosbag_filename: %s
      base_link_frame_id: "${MOLA_TF_BASE_LINK|base_link}"
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          sensorLabel: lidar
          fixed_sensor_pose: "${LIDAR_POSE_X|0} ${LIDAR_POSE_Y|0} ${LIDAR_POSE_Z|0} ${LIDAR_POSE_YAW|0} ${LIDAR_POSE_PITCH|0} ${LIDAR_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: '%s'
          type: CObservationIMU
          sensorLabel: imu
          fixed_sensor_pose: "${IMU_POSE_X|0} ${IMU_POSE_Y|0} ${IMU_POSE_Z|0} ${IMU_POSE_YAW|0} ${IMU_POSE_PITCH|0} ${IMU_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_IMU_POSE|false}
)"""",
    bagsYaml.c_str(), cli.arg_lidarTopic.c_str(), cli.arg_imuTopic.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_ROSBAG1)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_rosbag1(
  const Options & cli, const std::string & rosbag1file, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::Rosbag1Dataset>();
  o->setMinLoggingLevel(logLevel);

  // A comma-separated value becomes a YAML sequence, so a recording split
  // across several bag files (e.g. CitrusFarm) is replayed as the single
  // sequence it is. Rosbag1Dataset accepts a scalar or a sequence.
  std::string bagsYaml;
  {
    std::vector<std::string> parts;
    mrpt::system::tokenize(rosbag1file, ",", parts);
    ASSERT_(!parts.empty());
    if (parts.size() == 1) {
      bagsYaml = "'" + mrpt::system::trim(parts[0]) + "'";
    } else {
      for (const auto & p : parts) {
        bagsYaml += "\n        - '" + mrpt::system::trim(p) + "'";
      }
    }
  }

  // Same env var names as dataset_from_rosbag2() above and as
  // mola-lidar-odometry-cli's own dataset_from_rosbag1(), so the same
  // override snippet (and the same dataset profiles) work unchanged here.
  // Unlike that CLI's version, this carries only the lidar and imu entries:
  // Fast-LIO2 has no GNSS/wheel-odometry fusion path, it only ever reads
  // lidar_sensor_label/imu_sensor_label.
  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      rosbag_filename: %s
      base_link_frame_id: "${MOLA_TF_BASE_LINK|base_link}"
      sensors:
        - topic: '%s'
          type: CObservationPointCloud
          sensorLabel: lidar
          fixed_sensor_pose: "${LIDAR_POSE_X|0} ${LIDAR_POSE_Y|0} ${LIDAR_POSE_Z|0} ${LIDAR_POSE_YAW|0} ${LIDAR_POSE_PITCH|0} ${LIDAR_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_LIDAR_POSE|false}
        - topic: '%s'
          type: CObservationIMU
          sensorLabel: imu
          fixed_sensor_pose: "${IMU_POSE_X|0} ${IMU_POSE_Y|0} ${IMU_POSE_Z|0} ${IMU_POSE_YAW|0} ${IMU_POSE_PITCH|0} ${IMU_POSE_ROLL|0}"
          use_fixed_sensor_pose: ${MOLA_USE_FIXED_IMU_POSE|false}
)"""",
    bagsYaml.c_str(), cli.arg_lidarTopic.c_str(), cli.arg_imuTopic.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

#if defined(HAVE_MOLA_INPUT_KITTI)
std::shared_ptr<mola::OfflineDatasetSource> dataset_from_kitti(
  const std::string & kittiSeqNumber, const mrpt::system::VerbosityLevel logLevel)
{
  auto o = std::make_shared<mola::KittiOdometryDataset>();
  o->setMinLoggingLevel(logLevel);

  const auto cfg = mola::Yaml::FromText(mola::parse_yaml(mrpt::format(
    R""""(
    params:
      base_dir: ${KITTI_BASE_DIR}
      sequence: '%s'
      time_warp_scale: 1.0
      clouds_as_organized_points: false
      publish_lidar: true
      publish_image_0: false
      publish_image_1: false
      publish_ground_truth: true
)"""",
    kittiSeqNumber.c_str())));

  o->initialize(cfg);
  return o;
}
#endif

/// Writes to a file descriptor (the original standard output).
class FileDescriptorOutputStream : public mrpt::io::CStream
{
public:
  explicit FileDescriptorOutputStream(int fd) : f_(::fdopen(fd, "wb"))
  {
    if (f_ == nullptr) {
      THROW_EXCEPTION("Cannot open the standard output for writing");
    }
  }
  ~FileDescriptorOutputStream() override { std::fclose(f_); }
  FileDescriptorOutputStream(const FileDescriptorOutputStream &) = delete;
  FileDescriptorOutputStream & operator=(const FileDescriptorOutputStream &) = delete;

  size_t Read(void * /*buffer*/, size_t /*count*/) override
  {
    THROW_EXCEPTION("Write-only stream");
  }
  size_t Write(const void * buffer, size_t count) override
  {
    return std::fwrite(buffer, 1, count, f_);
  }
  uint64_t Seek(int64_t /*offset*/, TSeekOrigin /*origin*/) override
  {
    THROW_EXCEPTION("Cannot seek this stream");
  }
  uint64_t getTotalBytesCount() const override { return 0; }
  uint64_t getPosition() const override { return 0; }

private:
  FILE * f_ = nullptr;
};

/// A copy of a sensory frame whose point clouds carry their points. A
/// dataset may keep clouds in its own files and change them after loading
/// (e.g. KITTI's vertical angle correction), so a reference to the file
/// would not reproduce what the dataset reader returns.
mrpt::obs::CSensoryFrame::Ptr withPointsInline(const mrpt::obs::CSensoryFrame::Ptr & sf)
{
  using mrpt::obs::CObservationPointCloud;

  auto out = mrpt::obs::CSensoryFrame::Create();
  for (const auto & obs : *sf) {
    auto pc = std::dynamic_pointer_cast<CObservationPointCloud>(obs);
    if (pc && pc->isExternallyStored()) {
      pc->load();
      auto copy = std::make_shared<CObservationPointCloud>(*pc);
      copy->overrideExternalStorageFormatFlag(CObservationPointCloud::ExternalStorageFormat::None);
      out->insert(copy);
    } else {
      out->insert(obs);
    }
  }
  return out;
}

std::shared_ptr<mola::OfflineDatasetSource> openDataset(const Options & opt)
{
  using vl = mrpt::typemeta::TEnumType<mrpt::system::VerbosityLevel>;
  const auto logLevel = vl::name2value(opt.verbosity);

#if defined(HAVE_MOLA_INPUT_MULRAN)
  if (!opt.mulranSeq.empty()) {
    return dataset_from_mulran(opt.mulranSeq, logLevel);
  }
#endif
#if defined(HAVE_MOLA_INPUT_KITTI)
  if (!opt.kittiSeq.empty()) {
    return dataset_from_kitti(opt.kittiSeq, logLevel);
  }
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  if (!opt.rosbag2.empty()) {
    return dataset_from_rosbag2(opt, opt.rosbag2, logLevel);
  }
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG1)
  if (!opt.rosbag1.empty()) {
    return dataset_from_rosbag1(opt, opt.rosbag1, logLevel);
  }
#endif
  THROW_EXCEPTION("One dataset input option is required. Use --help.");
}

int run(const Options & opt)
{
  // With '-', the rawlog goes to the original standard output, and anything
  // else printed to it (e.g. dataset logs) is sent to the standard error.
  std::unique_ptr<mrpt::io::CStream> out;
  if (opt.output == "-") {
    std::fflush(stdout);
    const int dataFd = ::dup(STDOUT_FILENO);
    if (dataFd < 0 || ::dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
      THROW_EXCEPTION("Cannot redirect the standard output");
    }
    out = std::make_unique<FileDescriptorOutputStream>(dataFd);
  } else {
    out = std::make_unique<mrpt::io::CFileGZOutputStream>(opt.output);
  }

  const auto dataset = openDataset(opt);

  if (!opt.outputGt.empty()) {
    if (dataset->hasGroundTruthTrajectory()) {
      std::cerr << "Saving the ground truth trajectory to: " << opt.outputGt << "\n";
      dataset->getGroundTruthTrajectory().saveToTextFile_TUM(opt.outputGt);
    } else {
      std::cerr << "This dataset has no ground truth trajectory.\n";
    }
  }

  size_t n = dataset->datasetSize();
  if (opt.maxEntries > 0) {
    n = std::min(n, opt.maxEntries);
  }

  // One rawlog entry per dataset timestep, as a sensory frame.
  auto archive = mrpt::serialization::archiveFrom(*out);
  for (size_t i = 0; i < n; i++) {
    const auto sf = dataset->datasetGetObservations(i);
    if (sf) {
      archive << *withPointsInline(sf);
    } else {
      archive << *mrpt::obs::CSensoryFrame::Create();
    }
  }
  out.reset();

  std::cerr << "Done. Dataset entries written: " << n << "\n";
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  CLI::App app{"Writes a MOLA-supported dataset as an MRPT rawlog."};
  app
    .add_option(
      "-o,--output", opt.output,
      "Output rawlog file (gzip), or '-' for an uncompressed rawlog on the standard output")
    ->required();
  app.add_option(
    "--output-gt-tum", opt.outputGt, "Also save the ground truth, if any, to this TUM file");
  app.add_option("--only-first-n", opt.maxEntries, "Write only the first N dataset entries");
  app.add_option("-v,--verbosity", opt.verbosity, "Verbosity level: ERROR|WARN|INFO|DEBUG");

  auto * inputs = app.add_option_group("input", "Dataset input (exactly one)");
#if defined(HAVE_MOLA_INPUT_MULRAN)
  inputs->add_option(
    "--input-mulran-seq", opt.mulranSeq,
    "MulRan sequence (KAIST01, DCC01, ...), under ${MULRAN_BASE_DIR}");
#endif
#if defined(HAVE_MOLA_INPUT_KITTI)
  inputs->add_option(
    "--input-kitti-seq", opt.kittiSeq,
    "KITTI odometry sequence (00, 01, ...), under ${KITTI_BASE_DIR}");
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG2)
  inputs->add_option(
    "--input-rosbag2", opt.rosbag2,
    "ROS 2 bag (comma-separated list for a recording split in several bags)");
#endif
#if defined(HAVE_MOLA_INPUT_ROSBAG1)
  inputs->add_option(
    "--input-rosbag1", opt.rosbag1,
    "ROS 1 bag (comma-separated list for a recording split in several bags)");
#endif
  inputs->require_option(1);
#if defined(HAVE_MOLA_INPUT_ROSBAG2) || defined(HAVE_MOLA_INPUT_ROSBAG1)
  app.add_option("--lidar-topic", opt.arg_lidarTopic, "Bag input only: LiDAR point cloud topic");
  app.add_option("--imu-topic", opt.arg_imuTopic, "Bag input only: IMU topic");
#endif

  CLI11_PARSE(app, argc, argv);

  try {
    return run(opt);
  } catch (const std::exception & e) {
    mola::pretty_print_exception(e, "Exit due to exception:");
    return 1;
  }
}
