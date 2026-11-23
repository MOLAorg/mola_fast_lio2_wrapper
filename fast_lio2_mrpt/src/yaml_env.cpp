// SPDX-License-Identifier: BSD-3-Clause
// Copyright (C) 2026, Jose Luis Blanco-Claraco
// See LICENSES/BSD-3-Clause.txt and THIRD_PARTY_NOTICES.md.

#include <fast_lio2_mrpt/yaml_env.h>
#include <mrpt/core/exceptions.h>

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace fast_lio2_mrpt
{
std::string expandEnvVars(const std::string & text)
{
  std::string out;
  out.reserve(text.size());

  bool inComment = false;
  size_t i = 0;
  while (i < text.size()) {
    const char ch = text[i];
    if (ch == '\n') {
      inComment = false;
    } else if (ch == '#') {
      inComment = true;
    }

    const bool isVar = !inComment && ch == '$' && i + 1 < text.size() && text[i + 1] == '{';
    if (!isVar) {
      out += ch;
      i++;
      continue;
    }

    const size_t close = text.find('}', i + 2);
    if (close == std::string::npos) {
      THROW_EXCEPTION_FMT("Unterminated '${' at offset %zu", i);
    }
    const std::string expr = text.substr(i + 2, close - (i + 2));
    const size_t bar = expr.find('|');
    const std::string name = expr.substr(0, bar);

    if (const char * value = std::getenv(name.c_str()); value != nullptr) {
      out += value;
    } else if (bar != std::string::npos) {
      out += expr.substr(bar + 1);
    } else {
      THROW_EXCEPTION_FMT("Environment variable '%s' is not set and has no default", name.c_str());
    }
    i = close + 1;
  }
  return out;
}

mrpt::containers::yaml loadYamlFile(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    THROW_EXCEPTION_FMT("Cannot open YAML file '%s'", path.c_str());
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return mrpt::containers::yaml::FromText(expandEnvVars(ss.str()));
}

}  // namespace fast_lio2_mrpt
