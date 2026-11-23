# mola_fast_lio2_wrapper

Runs [Fast-LIO2](https://github.com/hku-mars/FAST_LIO) offline on
[MRPT](https://github.com/MRPT/mrpt) rawlogs and on any dataset that
[MOLA](https://github.com/MOLAorg/mola) can read, with the same interface as
[`mola-lidar-odometry-cli`](https://docs.mola-slam.org/latest/mola_lidar_odometry_cli.html),
in the same spirit as
[`mola_kiss_icp_wrapper`](https://github.com/MOLAorg/mola_kiss_icp_wrapper), so
Fast-LIO2 can be compared against other LiDAR odometry methods on the same data.

## Licenses

Fast-LIO2 and ikd-Tree are GPL-2.0-only, MOLA is GPL-3.0, and the two cannot
be combined into one program. This repository therefore builds two separate
programs, which only exchange MRPT rawlog data through a file or a pipe:

| Program | License | Links |
|---|---|---|
| `mola-fast-lio2-cli` | GPL-2.0-only | Fast-LIO2, ikd-Tree, MRPT. No MOLA library. |
| `mola-fast-lio2-dataset2rawlog` | GPL-3.0-only | MOLA dataset readers. No Fast-LIO2 code. |

This repository's own source code is BSD-3-Clause. See
[`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md) for the details, the
reasoning, and the upstream licenses.

## Programs

1. **`mola-fast-lio2-cli`**: processes an MRPT rawlog one entry at a time,
   each to completion before reading the next, so no scan is ever dropped,
   and saves the estimated trajectory in TUM format. Reads gzip, zstd or
   uncompressed rawlogs, or an uncompressed rawlog from its standard input
   (`--input-rawlog -`).
2. **`mola-fast-lio2-dataset2rawlog`**: writes a MOLA-supported dataset
   (MulRan, KITTI, ROS 1 and ROS 2 bags) as a rawlog, to a file or to its
   standard output. Built only if MOLA is found.

Both are built on `fast_lio2_core` (`third_party/fast_lio2_core/`), Fast-LIO2's
algorithm with every ROS dependency stripped out (plain C++ structs in and
out), and `fast_lio2_mrpt` (`fast_lio2_mrpt/`), which feeds it MRPT
observations.

## Usage

```bash
# MRPT rawlog:
mola-fast-lio2-cli -c pipelines/fastlio2-oxford-spires.yaml \
  --input-rawlog dataset.rawlog --output-tum-path out.tum

# A MOLA-supported dataset, through a pipe (or through a rawlog file):
MULRAN_BASE_DIR=/path/to/MulRan \
  mola-fast-lio2-dataset2rawlog --input-mulran-seq KAIST01 -o - | \
  mola-fast-lio2-cli -c pipelines/fastlio2-mulran.yaml \
    --input-rawlog - --output-tum-path kaist01.tum

# ROS 2 bag, with the LiDAR and IMU topics:
mola-fast-lio2-dataset2rawlog --input-rosbag2 bag_dir \
  --lidar-topic /hesai/pandar --imu-topic /alphasense_driver_ros/imu -o - | \
  mola-fast-lio2-cli -c pipelines/fastlio2-oxford-spires.yaml \
    --input-rawlog - --output-tum-path out.tum
```

For a bag without `/tf`, `mola-fast-lio2-dataset2rawlog` takes the same
environment variables for fixed sensor poses as `mola-lidar-odometry-cli`
(`MOLA_USE_FIXED_LIDAR_POSE`, `LIDAR_POSE_X`, ...). Add `--output-gt-tum
gt.tum` to save the dataset's ground truth, if it has one.

LiDAR-only datasets (KITTI's odometry benchmark, older MVSim rawlogs): add
`--synthetic-imu` to `mola-fast-lio2-cli`. It inserts a constant-gravity,
zero-rate IMU reading (sensor label `imu`) before each scan, and the motion
estimate then comes from the LiDAR alone. See `pipelines/fastlio2-mvsim.yaml`
and `pipelines/fastlio2-kitti.yaml`.

See `pipelines/*.yaml` for the Fast-LIO2 algorithm parameters (LiDAR-IMU
extrinsics, noise covariances, map cube size, etc.), one per supported
dataset and sensor. `${VAR}` and `${VAR|default}` in them are replaced with
environment variables.

## Build

```bash
cd ~/ros2_ws
git -C src clone --recursive https://github.com/MOLAorg/mola_fast_lio2_wrapper.git
colcon build --packages-select mola_fast_lio2_wrapper --cmake-args -DCMAKE_BUILD_TYPE=Release
```

`-DFASTLIO2_SYNC_IKD_REBUILD=ON` makes runs reproducible, bit for bit, at
some cost in speed (see `THIRD_PARTY_NOTICES.md`, ikd-Tree).

## Notes

- `fast_lio2_core` has no ROS or MOLA dependency, and is unit-tested
  standalone. It builds with `-DPCL_NO_PRECOMPILE` so `pcl::VoxelGrid` is
  instantiated from headers rather than linked from the system
  `libpcl_filters.so`: on at least one dev machine, the distro's precompiled
  PCL package and the installed Eigen headers disagreed on the
  aligned-allocator ABI, causing a heap-buffer-overflow on deallocation across
  the shared-library boundary. If you see similar corruption on a different
  platform even with this flag, that ABI mismatch is the first thing to check.
- Each rawlog entry feeds at most one observation: its first point cloud or,
  if it has none, its first IMU reading.
