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

#ifndef RMW_DDS_COMMON__TOPIC_CACHE_HPP_
#define RMW_DDS_COMMON__TOPIC_CACHE_HPP_

#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "rcpputils/thread_safety_annotations.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/types.h"

#include "rmw_dds_common/gid_utils.hpp"
#include "rmw_dds_common/visibility_control.h"

namespace rmw_dds_common
{

/**
 * Topic cache data structure. Manages relationships between participants and topics.
 */
class TopicCache
{
public:
  using TopicToTypes = std::unordered_map<std::string, std::vector<std::string>>;
  using NodeTopicMap = std::unordered_map<std::string, TopicToTypes>;
  using ParticipantNodeMap = std::map<rmw_gid_t, NodeTopicMap, Compare_rmw_gid_t>;

  /**
   * Getter for topic to types map.
   *
   * \return a map of topic name to the vector of topic types used.
   */
  RMW_DDS_COMMON_PUBLIC
  const TopicToTypes &
  get_topic_to_types() const;

  /**
   * Getter for participant to nodes to topics map.
   *
   * \return a map of participant gid to the vector of topic names used.
   */
  RMW_DDS_COMMON_PUBLIC
  const ParticipantNodeMap &
  get_participant_to_nodes_to_topics() const;

  /**
   * Add a topic based on discovery.
   *
   * /param gid
   * /param node_name
   * /param topic_name
   * /param type_name
   * /return true if a change has been recorded
   */
  RMW_DDS_COMMON_PUBLIC
  bool
  add_topic(
    const rmw_gid_t & gid,
    const std::string & node_name,
    const std::string & topic_name,
    const std::string & type_name);

  /**
   * Remove a topic based on discovery.
   *
   * /param gid
   * /param node_name
   * /param topic_name
   * /param type_name
   * /return true if a change has been recorded
   */
  RMW_DDS_COMMON_PUBLIC
  bool
  remove_topic(
    const rmw_gid_t & gid,
    const std::string & node_name,
    const std::string & topic_name,
    const std::string & type_name);

private:
  /**
   * Map of topic names to a vector of types that topic may use.
   * Topics here are represented as one to many, DDS XTypes 1.2
   * specifies application code 'generally' uses a 1-1 relationship.
   * However, generic services such as logger and monitor, can discover
   * multiple types on the same topic.
   */
  TopicToTypes topic_to_types_;

  /**
   * Map from participant gid to node names to topic names to a vector of topic types.
   */
  ParticipantNodeMap participant_to_nodes_to_topics_;

  /**
   * Helper function to initialize a topic vector.
   *
   * /param topic_name
   * /param topic_to_types
   */
  void
  initialize_topic(const std::string & topic_name, TopicToTypes & topic_to_types);

  /**
   * Helper function to initialize a NodeTopicMap.
   *
   * /param node_name
   * /param map
   */
  void
  initialize_node_topic_map(const std::string & node_name, NodeTopicMap & map);

  /**
   * Helper function to initialize the set inside a participant map.
   *
   * /param gid
   * /param map
   */
  void
  initialize_participant_node_map(const rmw_gid_t & gid, ParticipantNodeMap & map);
};

RMW_DDS_COMMON_PUBLIC
std::ostream &
operator<<(std::ostream & ostream, const TopicCache & topic_cache);

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__TOPIC_CACHE_HPP_
