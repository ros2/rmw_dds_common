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
#include <map>
#include <mutex>
#include <sstream>
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

#include "rmw_dds_common/gid_utils.hpp"
#include "rmw_dds_common/node_cache.hpp"
#include "rmw_dds_common/msg/gid.hpp"
#include "rmw_dds_common/msg/node_custom_info.hpp"
#include "rmw_dds_common/msg/participant_custom_info.hpp"

using rmw_dds_common::NodeCache;

rmw_ret_t
NodeCache::get_number_of_nodes(size_t & nodes_number) const
{
  size_t previous_nodes_number = 0;
  nodes_number = 0;
  std::lock_guard<std::mutex> guard(mutex_);
  for (const auto & elem : gid_to_node_info_vector_) {
    nodes_number += elem.second.size();
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
    for (const auto & elem : gid_to_node_info_vector_) {
      const auto & nodes_info = elem.second;
      for (const auto & node_info : nodes_info) {
        node_names->data[j] = rcutils_strdup(node_info.node_name.c_str(), *allocator);
        node_namespaces->data[j] = rcutils_strdup(
          node_info.node_namespace.c_str(),
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

void
NodeCache::update_node_names(
  const rmw_gid_t & gid,
  const NodeInfoVector & node_info_vector)
{
  std::lock_guard<std::mutex> guard(mutex_);
  gid_to_node_info_vector_[gid] = node_info_vector;
}

rmw_ret_t
NodeCache::add_gid(const rmw_gid_t & gid)
{
  std::lock_guard<std::mutex> guard(mutex_);
  const auto & it = gid_to_node_info_vector_.find(gid);
  if (it != gid_to_node_info_vector_.end()) {
    return RMW_RET_ERROR;
  }
  auto ret = gid_to_node_info_vector_.emplace(
    std::piecewise_construct,
    std::forward_as_tuple(gid),
    std::forward_as_tuple());
  return ret.second ? RMW_RET_OK : RMW_RET_ERROR;
}

rmw_ret_t
NodeCache::add_node_name(
  const rmw_gid_t & gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  std::lock_guard<std::mutex> guard(mutex_);
  const auto & it = gid_to_node_info_vector_.find(gid);
  if (it == gid_to_node_info_vector_.end()) {
    return RMW_RET_ERROR;
  }
  it->second.emplace_back();
  it->second.back().node_name = node_name;
  it->second.back().node_namespace = node_namespace;
  return RMW_RET_OK;
}

rmw_ret_t
NodeCache::delete_node_name(
  const rmw_gid_t & gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  std::lock_guard<std::mutex> guard(mutex_);
  auto it = gid_to_node_info_vector_.find(gid);
  if (it == gid_to_node_info_vector_.end()) {
    return RMW_RET_ERROR;
  }
  it->second.erase(
    std::remove_if(
      it->second.begin(),
      it->second.end(),
      [&node_namespace, &node_name](const rmw_dds_common::msg::NodeCustomInfo & elem) {
        return elem.node_namespace == node_namespace && elem.node_name == node_name;
      }),
    it->second.end());
  return RMW_RET_OK;
}

rmw_ret_t
NodeCache::get_participant_state_message(
  const rmw_gid_t & gid,
  rmw_dds_common::msg::ParticipantCustomInfo & participant_info) const
{
  convert_gid_to_msg(&gid, &participant_info.id);
  {
    std::lock_guard<std::mutex> guard(mutex_);
    const auto & it = gid_to_node_info_vector_.find(gid);
    if (it == gid_to_node_info_vector_.end()) {
      return RMW_RET_ERROR;
    }
    participant_info.nodes_info.reserve(it->second.size());
    for (const auto & node_info : it->second) {
      participant_info.nodes_info.emplace_back();
      participant_info.nodes_info.back().node_name = node_info.node_name;
      participant_info.nodes_info.back().node_namespace = node_info.node_namespace;
    }
  }
  return RMW_RET_OK;
}

bool
NodeCache::delete_node_names(const rmw_gid_t & gid)
{
  std::lock_guard<std::mutex> guard(mutex_);
  return gid_to_node_info_vector_.erase(gid) != 0;
}

std::ostream &
rmw_dds_common::operator<<(std::ostream & ostream, const NodeCache & node_cache)
{
  std::ostringstream ss;
  ss << "Node namespaces and names:" << std::endl;
  for (const auto & elem : node_cache.gid_to_node_info_vector_) {
    ss << "  gid: " << elem.first << std::endl;
    for (const auto & node_info : elem.second) {
      ss << "    namespace=[" << node_info.node_namespace << "] name=[" <<
        node_info.node_name << "]" << std::endl;
    }
  }
  return ostream << ss.str();
}
