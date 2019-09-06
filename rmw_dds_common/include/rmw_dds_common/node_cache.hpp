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

#ifndef RMW_DDS_COMMON__NODE_CACHE_HPP_
#define RMW_DDS_COMMON__NODE_CACHE_HPP_

#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/thread_safety_annotations.hpp"
#include "rcutils/allocator.h"
#include "rcutils/error_handling.h"
#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"
#include "rcutils/types.h"
#include "rcutils/types/string_array.h"
#include "rmw/error_handling.h"
#include "rmw/ret_types.h"
#include "rmw/sanity_checks.h"
#include "rmw/types.h"

namespace rmw_dds_common
{

/**
 * Class that subscribes to the topic where participants publish its node, and
 * updates a data structure.
 */
class NodeCache
{
public:
  using NodeNames = std::pair<std::vector<std::string>, std::vector<std::string>>;
  using GuidToNodeNames = std::map<GUID_t, NodeNames>;

  /**
   * Get the number of nodes that have been discovered.
   * \return RMW_RET_OK, or
   * \return RMW_RET_ERROR
   */
  rmw_ret_t
  get_number_of_nodes(size_t & nodes_number) const
  {
    size_t previous_nodes_number = 0;
    nodes_number = 0;
    std::lock_guard<std::mutex> guard(mutex_);
    for (const auto & elem : GUID_to_node_names_) {
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

  /**
   * Copies the names and namespaces of the discovered nodes.
   *
   * \return RMW_RET_OK, or
   * \return RMW_RET_INVALID_ARGUMENT, or
   * \return RMW_RET_BAD_ALLOC, or
   * \return RMW_RET_ERROR
   */
  rmw_ret_t
  get_node_names(
    rcutils_string_array_t * node_names,
    rcutils_string_array_t * node_namespaces,
    rcutils_allocator_t * allocator) const
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
      size_t j = 0;
      for (const auto & elem : GUID_to_node_names_) {
        const auto names_and_namespaces = elem.second;
        for (size_t i = 0; i < names_and_namespaces.first.size(); i++) {
          node_names->data[j] = rcutils_strdup(names_and_namespaces.first[i].c_str(), *allocator);
          node_namespaces->data[j] = rcutils_strdup(
            names_and_namespaces.second[i].c_str(),
            *allocator);
          j++;
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

  /**
   * Updates the stored node names of a specific guid with new data.
   */
  void
  update_node_names(
    GUID_t guid,
    std::vector<std::string> node_names,
    std::vector<std::string> node_namespaces)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    auto & names_namespaces = GUID_to_node_names_[guid];
    names_namespaces.first = node_names;
    names_namespaces.second = node_namespaces;
  }

  /**
   * Updates the stored node names of a specific guid with new data.
   *
   * \return `true` if existing data was deleted, or
   * \return `false` if there were not stored data for the given guid.
   */
  bool
  delete_node_names(GUID_t guid)
  {
    std::lock_guard<std::mutex> guard(mutex_);
    return GUID_to_node_names_.erase(guid) != 0;
  }

private:
  mutable std::mutex mutex_;
  GuidToNodeNames GUID_to_node_names_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__NODE_CACHE_HPP_
