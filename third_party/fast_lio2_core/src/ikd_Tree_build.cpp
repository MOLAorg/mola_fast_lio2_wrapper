// SPDX-License-Identifier: BSD-3-Clause
// Copyright (C) 2026, Jose Luis Blanco-Claraco
//
// Builds the ikd-Tree sources (third_party/ikd-Tree, upstream, unmodified,
// GPL-2.0-only; see THIRD_PARTY_NOTICES.md) as part of fast_lio2_core.
//
// ikd-Tree hands any subtree larger than Multi_Thread_Rebuild_Point_Num to a
// background rebuild thread, so results depend on thread timing. With
// FASTLIO2_SYNC_IKD_REBUILD defined, the threshold is raised past any
// reachable subtree size, so every rebuild runs inline and runs are
// reproducible. The macro is only used in ikd_Tree.cpp, and ikd_Tree.h has
// `#pragma once`, so redefining it between the two includes below is enough.

#include <ikd_Tree.h>

#if defined(FASTLIO2_SYNC_IKD_REBUILD)
#undef Multi_Thread_Rebuild_Point_Num
#define Multi_Thread_Rebuild_Point_Num 2000000000
#endif

#include "ikd_Tree.cpp"  // NOLINT(bugprone-suspicious-include)
