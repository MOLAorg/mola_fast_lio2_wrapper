// SPDX-License-Identifier: BSD-3-Clause
// Copyright (C) 2026, Jose Luis Blanco-Claraco
// See LICENSES/BSD-3-Clause.txt and THIRD_PARTY_NOTICES.md.

#include <fast_lio2_mrpt/yaml_env.h>
#include <gtest/gtest.h>

#include <cstdlib>
#include <stdexcept>

using fast_lio2_mrpt::expandEnvVars;

namespace
{
// Unlikely to be set in any test environment.
constexpr const char * kUnset = "FASTLIO2_TEST_SURELY_UNSET_VAR";
constexpr const char * kSet = "FASTLIO2_TEST_SET_VAR";
}  // namespace

TEST(YamlEnv, SetVariableWinsOverDefault)
{
  ::setenv(kSet, "60.0", 1);
  EXPECT_EQ(expandEnvVars("det_range: ${FASTLIO2_TEST_SET_VAR}\n"), "det_range: 60.0\n");
  EXPECT_EQ(expandEnvVars("a: ${FASTLIO2_TEST_SET_VAR|1.0}"), "a: 60.0");
  ::unsetenv(kSet);
}

TEST(YamlEnv, DefaultsWhenUnset)
{
  ::unsetenv(kUnset);
  EXPECT_EQ(expandEnvVars("a: ${FASTLIO2_TEST_SURELY_UNSET_VAR|0.5}"), "a: 0.5");
  EXPECT_EQ(expandEnvVars("a: '${FASTLIO2_TEST_SURELY_UNSET_VAR|}'"), "a: ''");
  EXPECT_EQ(
    expandEnvVars("f: ${FASTLIO2_TEST_SURELY_UNSET_VAR|'out.tum'}\n"
                  "p: \"${FASTLIO2_TEST_SURELY_UNSET_VAR|0 0 0.75 0 0 0}\""),
    "f: 'out.tum'\np: \"0 0 0.75 0 0 0\"");
}

TEST(YamlEnv, UnsetWithoutDefaultThrows)
{
  ::unsetenv(kUnset);
  EXPECT_THROW(expandEnvVars("a: ${FASTLIO2_TEST_SURELY_UNSET_VAR}"), std::exception);
  EXPECT_THROW(expandEnvVars("a: ${NOT_CLOSED"), std::exception);
}

TEST(YamlEnv, CommentsAreLeftAlone)
{
  ::unsetenv(kUnset);
  const std::string text =
    "# uses ${FASTLIO2_TEST_SURELY_UNSET_VAR}\n"
    "a: 1  # ${FASTLIO2_TEST_SURELY_UNSET_VAR}\n"
    "b: ${FASTLIO2_TEST_SURELY_UNSET_VAR|2}\n";
  EXPECT_EQ(
    expandEnvVars(text),
    "# uses ${FASTLIO2_TEST_SURELY_UNSET_VAR}\n"
    "a: 1  # ${FASTLIO2_TEST_SURELY_UNSET_VAR}\n"
    "b: 2\n");
}
