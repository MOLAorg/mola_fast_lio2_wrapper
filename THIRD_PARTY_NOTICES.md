# Licensing and third-party notices

This repository runs **Fast-LIO2**, a LiDAR-inertial odometry algorithm by the
MARS Lab, The University of Hong Kong, on datasets read with
[MRPT](https://github.com/MRPT/mrpt) and [MOLA](https://github.com/MOLAorg/mola).
It contains code under three licenses, and each source file states its own in
an `SPDX-License-Identifier` line:

| What | License | Text |
|---|---|---|
| The `mola-fast-lio2-cli` program, as a whole (it contains Fast-LIO2 and ikd-Tree) | GPL-2.0-only | [`LICENSE`](LICENSE) |
| Files derived from Fast-LIO2, and ikd-Tree, under `third_party/` (details below) | GPL-2.0-only (two files also carry a BSD-3-Clause notice) | [`LICENSE`](LICENSE) |
| IKFoM toolkit, `third_party/IKFoM_toolkit/` | BSD-3-Clause (HKU) | its file headers |
| This repository's own code: `fast_lio2_mrpt/`, `apps/mola-fast-lio2-cli.cpp`, `third_party/fast_lio2_core/src/ikd_Tree_build.cpp`, tests, build files | BSD-3-Clause | [`LICENSES/BSD-3-Clause.txt`](LICENSES/BSD-3-Clause.txt) |
| The `mola-fast-lio2-dataset2rawlog` program (`apps/mola-fast-lio2-dataset2rawlog.cpp`), which links MOLA | GPL-3.0-only | [`LICENSES/GPL-3.0-only.txt`](LICENSES/GPL-3.0-only.txt) |

## Why the programs are split this way

Fast-LIO2 and ikd-Tree are GPL-2.0-only, and MOLA is GPL-3.0. The Free
Software Foundation considers these two licenses incompatible: code under
GPLv2 only and code under GPLv3 cannot be combined into one program
([GPL FAQ, "Is GPLv3 compatible with GPLv2?"](https://www.gnu.org/licenses/gpl-faq.html#v2v3Compatibility)).
Linking, either static or dynamic, makes one combined work
([GPL FAQ, static vs. dynamic linking](https://www.gnu.org/licenses/gpl-faq.html#GPLStaticVsDynamic)),
and so does a plug-in that a program loads and shares data structures with
([GPL FAQ, plug-ins](https://www.gnu.org/licenses/gpl-faq.html#GPLPlugins)).
Hence:

- **`mola-fast-lio2-cli` links no GPL-3.0 code.** It is built from
  `fast_lio2_core` (GPL-2.0-only), this repository's BSD-3-Clause code, and
  libraries whose licenses are compatible with GPLv2: MRPT and PCL
  (BSD-3-Clause), Eigen (MPL-2.0, compatible through its section 3.3), Boost
  (BSL-1.0), CLI11 (BSD-3-Clause), and the compiler runtime libraries. The
  program as a whole is distributed under GPL-2.0-only. The few MOLA features
  it used (reading a rawlog, expanding `${VAR|default}` in YAML files) are
  implemented here, from scratch, on top of MRPT.
- **MOLA dataset readers run in a separate program.**
  `mola-fast-lio2-dataset2rawlog` (GPL-3.0-only, no Fast-LIO2 code) reads
  MulRan, KITTI and ROS 1/ROS 2 bags with MOLA, and writes them as an MRPT
  rawlog, to a file or to its standard output. The two programs exchange
  nothing but that public file format, through a file or a pipe: the FSF
  regards pipes and files as communication between separate programs
  ([GPL FAQ, aggregates](https://www.gnu.org/licenses/gpl-faq.html#MereAggregation)),
  and both licenses allow separate programs to be distributed together
  (GPLv2 section 2, last paragraph; GPLv3 section 5, last paragraph).
- **There is no `mola-cli` module.** A module loaded into the `mola-cli`
  process would be part of one program with MOLA, so it cannot contain
  Fast-LIO2.

Do not add a dependency on any MOLA library (or other GPL-3.0 code) to
`fast_lio2_core`, `fast_lio2_mrpt` or `mola-fast-lio2-cli`.

## Fast-LIO2

- Upstream: https://github.com/hku-mars/FAST_LIO (branch `ROS2`)
- License: **GNU General Public License v2.0 only** (no "or later" clause:
  the repository's `LICENSE` file is the GPLv2 text, and no file states an
  "or later" grant).
- Copyright: the FAST-LIO2 authors, MARS Lab, The University of Hong Kong
  (contributors listed in its README: Wei Xu, Yixi Cai, Dongjiao He,
  Fangcheng Zhu, Jiarong Lin, Zheng Liu, Borong Yuan).
- `laserMapping.cpp` also carries a BSD-3-Clause notice from its LOAM
  lineage: Copyright 2013 Ji Zhang, Carnegie Mellon University; further
  contributions Copyright (c) 2016 Southwest Research Institute; modified by
  Livox. The files derived from it (`FastLio2Core.cpp` and `FastLio2Core.h`)
  retain that notice verbatim, and are `GPL-2.0-only AND BSD-3-Clause`.
- Files derived from it in this repository, all under
  `third_party/fast_lio2_core/`:
  - `include/fast_lio2_core/ImuProcess.h` and `src/ImuProcess.cpp`: ported
    from `IMU_Processing.hpp` (ROS message types and `rclcpp::Time` replaced
    with plain structs; algorithm logic unchanged).
  - `src/FastLio2Core.cpp` and `include/fast_lio2_core/FastLio2Core.h`: the
    per-scan processing steps of `laserMapping.cpp`'s main loop, moved into a
    stateful class.
  - `include/fast_lio2_core/Types.hpp`: plain data types, partly adapted from
    `common_lib.h` (`MeasureGroup`, `Pose6D`, `set_pose6d()`, constants).
  - `include/fast_lio2_core/so3_math.h`: `include/so3_math.h`, verbatim.
  - `include/fast_lio2_core/use-ikfom.hpp`: `include/use-ikfom.hpp` (the
    Fast-LIO2 state on top of IKFoM), with one mechanical change: its free
    functions are marked `inline`, since this repository includes it from
    more than one translation unit (an ODR link error otherwise). No
    algorithmic change.

## ikd-Tree

- Upstream: https://github.com/hku-mars/ikd-Tree, branch `fast_lio`, commit
  `e2e3f4e`: the same commit that FAST_LIO's `ROS2` branch uses for its own
  `include/ikd-Tree` submodule.
- Used as a git submodule at `third_party/ikd-Tree`, **unmodified**.
- License: **GNU General Public License v2.0 only**. The `fast_lio` branch
  carries no license file and its source files no license notice. The
  license is stated for the whole repository on its default branch, `main`:
  the GPLv2 text in its `LICENSE` file (added in commit `81eaeee`,
  2021-07-29; identical to this repository's `LICENSE`), and its README:
  "The source code of ikd-Tree is released under GPLv2 license", without an
  "or later" option. GitHub's metadata for the repository reports GPL-2.0.
  The same files are also distributed as part of FAST_LIO, under its GPLv2
  `LICENSE`.
- Copyright: the ikd-Tree authors (Yixi Cai, Wei Xu, Fu Zhang, MARS Lab, The
  University of Hong Kong).
- The CMake option `FASTLIO2_SYNC_IKD_REBUILD` (default `OFF`) makes runs
  reproducible: ikd-Tree hands large subtrees to a background rebuild thread,
  so results depend on thread timing. With the option on,
  `third_party/fast_lio2_core/src/ikd_Tree_build.cpp` (this repository's
  code) raises `Multi_Thread_Rebuild_Point_Num` past any reachable subtree
  size and then compiles the upstream `ikd_Tree.cpp`, so every rebuild runs
  inline. No upstream file is changed.

## IKFoM (Iterated Kalman Filter on Manifolds) toolkit

- Bundled inside the FAST-LIO2 repository at `include/IKFoM_toolkit/`.
- License: **BSD 3-Clause**.
- Copyright (c) 2019-2023, The University of Hong Kong. Author: Dongjiao HE
  (hdj65822@connect.hku.hk). (IKFoM is itself a fork of the MTK manifold
  toolkit; some files retain an older University of Bremen attribution from
  that lineage.)
- Vendored at `third_party/IKFoM_toolkit`, **unmodified**.
