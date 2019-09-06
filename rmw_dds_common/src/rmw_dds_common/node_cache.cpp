// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <limits>
#include <string>
#include <vector>

#include "node_cache.hpp"
#include "rmw/error_handling"


namespace rmw_dds_common
{

rmw_ret_t
NodeCache::get_number_of_nodes(size_t & nodes_number)
{
  size_t previous_nodes_number = 0;
  nodes_number = 0;
  std::lock_guard<std::mutex> guard(mutex_);
  for (const auto & elem : guid_to_node_names_) {
    nodes_number += elem.second.first.size();
    if (nodes_number < previous_nodes_number) {
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "Discovered more than %zu nodes",
        std::numeric_limits<size_t>::max());
      return RMW_RET_ERROR;
    }
    previous_nodes_number = nodes_number;
  }
  return RMW_RET_OK;
}

rmw_ret_t
NodeCache::get_node_names(
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces
  rcutils_allocator_t * allocator)
{
  if (rmw_check_zero_rmw_string_array(node_names) != RMW_RET_OK) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (rmw_check_zero_rmw_string_array(node_namespaces) != RMW_RET_OK) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(allocator, "get_node_names allocator is not valid",
    return RMW_RET_INVALID_ARGUMENT);

  size_t nodes_number;
  if (RMW_RET_OK != get_number_of_nodes(nodes_number)) {
    return RMW_RET_ERROR;
  }
  rcutils_ret_t rcutils_ret =
    rcutils_string_array_init(node_names, nodes_number, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
    rcutils_reset_error();
    goto fail;
  }
  rcutils_ret =
    rcutils_string_array_init(node_namespaces, nodes_number, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
    rcutils_reset_error();
    goto fail;
  }
  {
    std::lock_guard<std::mutex> guard(mutex_);
    for (const auto & elem : guid_to_node_names_) {
      const auto names_and_namespaces = elem.second;
      for (size_t i = 0; i < names_and_namespaces.first.size(); i++) {
        node_names->data[i] = rcutils_strdup(names_and_namespaces.first, *allocator);
        node_namespaces->data[i] = rcutils_strdup(names_and_namespaces.second, *allocator);
      }
    }
  }
  return RMW_RET_OK;

fail:
  rcutils_ret = rcutils_string_array_fini(node_names);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_dds_common",
      "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
  rcutils_ret = rcutils_string_array_fini(node_namespaces);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_dds_common",
      "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
    rcutils_reset_error();
  }
  return RMW_RET_BAD_ALLOC;
}

void
NodeCache::update_node_names(
  Guid_t guid,
  std::vector<std::string> node_names,
  std::vector<std::string> node_namespaces)
{
  std::lock_guard<std::mutex> guard(mutex_);
  auto & names_namespaces = guid_to_node_names_[guid];
  names_namespaces.first = node_names;
  names_namespaces.second = node_namespaces;
}

bool
NodeCache::delete_node_names(Guid_t guid)
{
  std::lock_guard<std::mutex> guard(mutex_);
  return guid_to_node_names_.erase(guid) != 0;
}

}  // namespace rmw_dds_common
