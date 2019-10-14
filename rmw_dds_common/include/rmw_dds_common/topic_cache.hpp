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
#include <utility>
#include <vector>

#include "rcpputils/thread_safety_annotations.hpp"
#include "rcutils/logging_macros.h"
#include "rmw/names_and_types.h"
#include "rmw/types.h"

#include "rmw_dds_common/gid_utils.hpp"
#include "rmw_dds_common/visibility_control.h"

namespace rmw_dds_common
{

class StringPairHash
{
public:
  template<typename T>
  inline void hash_combine(std::size_t & seed, const T & v) const
  {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }

  inline size_t operator()(const std::pair<std::string, std::string> & s) const
  {
    size_t seed = 0;
    hash_combine(seed, s.first);
    hash_combine(seed, s.second);
    return seed;
  }
};

/**
 * Topic cache data structure. Manages relationships between participants and topics.
 */
class TopicCache
{
public:
  using NamespaceNamePair = std::pair<std::string, std::string>;
  using TopicToTypes = std::unordered_map<std::string, std::vector<std::string>>;
  using NodeTopicMap = std::unordered_map<NamespaceNamePair, TopicToTypes, StringPairHash>;
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
   * /param namespace_
   * /param node_name
   * /param topic_name
   * /param type_name
   * /return true if a change has been recorded
   */
  RMW_DDS_COMMON_PUBLIC
  bool
  add_topic(
    const rmw_gid_t & gid,
    const std::string & namespace_,
    const std::string & node_name,
    const std::string & topic_name,
    const std::string & type_name);

  /**
   * Remove a topic based on discovery.
   *
   * /param gid
   * /param namespace_
   * /param node_name
   * /param topic_name
   * /param type_name
   * /return true if a change has been recorded
   */
  RMW_DDS_COMMON_PUBLIC
  bool
  remove_topic(
    const rmw_gid_t & gid,
    const std::string & namespace_,
    const std::string & node_name,
    const std::string & topic_name,
    const std::string & type_name);

  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_count(std::string topic_name, std::string (* mangle_topic)(std::string), size_t * count);

  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_names_and_types_by_node(
    const rmw_gid_t & gid,
    const std::string & node_name,
    const std::string & namespace_,
    std::string (* demangle_topic)(const std::string &),
    std::string (* demangle_type)(const std::string &),
    rcutils_allocator_t * allocator,
    rmw_names_and_types_t * topic_names_and_types);

  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_names_and_types(
    std::string (* demangle_topic)(const std::string &),
    std::string (* demangle_type)(const std::string &),
    rcutils_allocator_t * allocator,
    rmw_names_and_types_t * topic_names_and_types);

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
   * /param name_pair
   * /param map
   */
  void
  initialize_node_topic_map(const NamespaceNamePair & name_pair, NodeTopicMap & map);

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
