// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <filesystem>
#include <fstream>
#include <functional>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#include "rmw_dds_common/security.hpp"
#include "rmw_security_common/security.hpp"
#include "rmw/error_handling.h"

#include "rcpputils/scope_exit.hpp"

#include "rcutils/allocator.h"
#include "rcutils/types/string_map.h"

namespace rmw_dds_common
{

bool get_security_files(
  const std::string & prefix, const std::string & secure_root,
  std::unordered_map<std::string, std::string> & result)
{
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
# ifdef __clang__
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wdeprecated-declarations"
# endif
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
  return get_security_files(false, prefix, secure_root, result);
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif
}

bool get_security_files(
  bool supports_pkcs11,
  const std::string & prefix,
  const std::string & secure_root,
  std::unordered_map<std::string, std::string> & result)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcutils_string_map_t security_files = rcutils_get_zero_initialized_string_map();
  rcutils_ret_t ret = rcutils_string_map_init(&security_files, 0, allocator);

  auto scope_exit_ws = rcpputils::make_scope_exit(
    [&security_files]()
    {
      rcutils_ret_t ret = rcutils_string_map_fini(&security_files);
      if (ret != RMW_RET_OK) {
        RMW_SET_ERROR_MSG("error cleaning string map memory");
      }
    });

  if (ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG("error initializin map");
    return false;
  }

  ret = get_security_files_support_pkcs(
    supports_pkcs11, prefix.c_str(), secure_root.c_str(), &security_files);
  if (ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG("error calling get_security_files_support_pkcs");
    return false;
  }

  const char * key = rcutils_string_map_get_next_key(&security_files, NULL);
  while (key != NULL) {
    const char * value = rcutils_string_map_get(&security_files, key);
    if (NULL == value) {
      RMW_SET_ERROR_MSG("unable to get value for known key, should not happen");
      return false;
    }
    result[key] = value;
    key = rcutils_string_map_get_next_key(&security_files, key);
  }

  return true;
}

}  // namespace rmw_dds_common
