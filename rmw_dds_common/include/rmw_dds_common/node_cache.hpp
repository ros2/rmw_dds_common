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

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "rcpputils/thread_safety_annotations.hpp"
#include "rcutils/allocator.h"
#include "rcutils/types/string_array.h"
#include "rmw/ret_types.h"
#include "rmw/types.h"

#include "rmw_dds_common/gid_utils.hpp"
#include "rmw_dds_common/visibility_control.h"
#include "rmw_dds_common/msg/gid.hpp"
#include "rmw_dds_common/msg/node_custom_info.hpp"
#include "rmw_dds_common/msg/participant_custom_info.hpp"

namespace rmw_dds_common
{

/**
 * Topic cache data structure. Manages relationships between participants and nodes.
 */
class NodeCache
{
public:
  using NodeInfoVector = std::vector<rmw_dds_common::msg::NodeCustomInfo>;
  using GidToNodeInfoVector = std::map<
    rmw_gid_t,
    std::vector<rmw_dds_common::msg::NodeCustomInfo>,
    Compare_rmw_gid_t>;

  /**
   * Get the number of nodes that have been discovered.
   * \return RMW_RET_OK, or
   * \return RMW_RET_ERROR
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_number_of_nodes(size_t & nodes_number) const;

  /**
   * Copy the names and namespaces of the discovered nodes.
   *
   * \return RMW_RET_OK, or
   * \return RMW_RET_INVALID_ARGUMENT, or
   * \return RMW_RET_BAD_ALLOC, or
   * \return RMW_RET_ERROR
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_node_names(
    rcutils_string_array_t * node_names,
    rcutils_string_array_t * node_namespaces,
    rcutils_allocator_t * allocator) const;

  /**
   * Updates the stored node names of a specific gid with new data.
   */
  RMW_DDS_COMMON_PUBLIC
  void
  update_node_names(
    const rmw_gid_t & gid,
    const NodeInfoVector & node_info_vector);

  /**
   * Add a new Participant gid.
   *
   * \return RMW_RET_ERROR if it was already added, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  add_gid(const rmw_gid_t & gid);

  /**
   * Add a new node name for a specific Participant gid.
   *
   * \return RMW_RET_ERROR if the participant gid doesn't exist, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  add_node_name(
    const rmw_gid_t & gid,
    const std::string & node_name,
    const std::string & node_namespace);

  /**
   * Generate a message from existing participant data.
   *
   * \return RMW_RET_ERROR if the participant gid doesn't exist, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_participant_state_message(
    const rmw_gid_t & gid,
    rmw_dds_common::msg::ParticipantCustomInfo & participant_info) const;

  /**
   * Updates the stored node names of a specific gid with new data.
   *
   * \return `true` if existing data was deleted, or
   * \return `false` if there were not stored data for the given gid.
   */
  RMW_DDS_COMMON_PUBLIC
  bool
  delete_node_names(const rmw_gid_t & gid);

private:
  mutable std::mutex mutex_;
  GidToNodeInfoVector gid_to_node_info_vector_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
};

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__NODE_CACHE_HPP_
