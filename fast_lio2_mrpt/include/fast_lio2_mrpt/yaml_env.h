// SPDX-License-Identifier: BSD-3-Clause
// Copyright (C) 2026, Jose Luis Blanco-Claraco
// See LICENSES/BSD-3-Clause.txt and THIRD_PARTY_NOTICES.md.

/**
 * @file   yaml_env.h
 * @brief  Environment variable expansion for the pipeline YAML files.
 */
#pragma once

#include <mrpt/containers/yaml.h>

#include <string>

namespace fast_lio2_mrpt
{
/** Expands environment variables in a text:
 *  - `${NAME}` becomes the value of NAME, which must be set.
 *  - `${NAME|default}` becomes the value of NAME, or `default` if NAME is
 *    not set (`default` may be empty).
 *
 * Text from a `#` to the end of its line is a comment and is copied as is.
 * Throws if a variable without default is not set, or a `${` is not closed.
 */
std::string expandEnvVars(const std::string & text);

/** Reads a YAML file, expanding environment variables first (see
 * expandEnvVars()). */
mrpt::containers::yaml loadYamlFile(const std::string & path);

}  // namespace fast_lio2_mrpt
