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

#ifndef RMW_DDS_COMMON__GRAPH_CACHE_HPP_
#define RMW_DDS_COMMON__GRAPH_CACHE_HPP_

#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcutils/logging_macros.h"
#include "rmw/names_and_types.h"
#include "rmw/types.h"

#include "rmw_dds_common/gid_utils.hpp"
#include "rmw_dds_common/visibility_control.h"
#include "rmw_dds_common/msg/gid.hpp"
#include "rmw_dds_common/msg/node_entities_info.hpp"
#include "rmw_dds_common/msg/participant_entities_info.hpp"

namespace rmw_dds_common
{

// Forward-declarations of things at the end of the file
class StringPairHash;
struct EntityInfo;

/**
 * Graph cache data structure.
 *
 * Manages relationships between participants, nodes and topics.
 */
class GraphCache
{
  friend
  std::ostream &
  operator<<(std::ostream & ostream, const GraphCache & topic_cache);

public:
  /**
   * \defgroup dds_discovery_api
   * Methods used to update the Graph Cache based on DDS discovery.
   * @{
   */

  /**
   * Add a data writer based on discovery.
   *
   * /param gid the data writer guid
   * /param topic_name
   * /param type_name
   * /return true if a change has been recorded
   */
  RMW_DDS_COMMON_PUBLIC
  bool
  add_writer(
    const rmw_gid_t & gid,
    const std::string & topic_name,
    const std::string & type_name);

  /**
   * Add a data reader based on discovery.
   *
   * /param gid the data reader guid
   * /param topic_name
   * /param type_name
   * /return true if a change has been recorded
   */
  RMW_DDS_COMMON_PUBLIC
  bool
  add_reader(
    const rmw_gid_t & gid,
    const std::string & topic_name,
    const std::string & type_name);

  RMW_DDS_COMMON_PUBLIC
  bool
  add_entity(
    const rmw_gid_t & gid,
    const std::string & topic_name,
    const std::string & type_name,
    bool is_reader);

  /**
   * Remove a data writer based on discovery.
   *
   * /param gid the data writer guid
   * /return true if a change has been recorded
   */
  RMW_DDS_COMMON_PUBLIC
  bool
  remove_writer(const rmw_gid_t & gid);

  /**
   * Remove a data reader based on discovery.
   *
   * /param gid the data reader guid
   * /return true if a change has been recorded
   */
  RMW_DDS_COMMON_PUBLIC
  bool
  remove_reader(const rmw_gid_t & gid);

  RMW_DDS_COMMON_PUBLIC
  bool
  remove_entity(const rmw_gid_t & gid, bool is_reader);

  /**
   * @}
   * \defgroup common_api
   * Methods used to update the Graph Cache either based on DDS discovery or
   * in local entities changes.
   * @{
   */

  /**
   * Remove a participant based on discovery.
   *
   * /param participant_gid
   * /return true if a change has been recorded
   */
  RMW_DDS_COMMON_PUBLIC
  bool
  remove_participant(const rmw_gid_t & participant_gid);

  /**
   * @}
   * \defgroup ros_discovery_api
   * Methods used to update the Graph Cache based on ROS 2 specific messages received from
   * remote participants.
   * @{
   */

  /**
   * Update participant info, based on received ParticipantEntitiesInfo message.
   *
   * /param msg will be filled with the received participant entities info message
   */
  RMW_DDS_COMMON_PUBLIC
  void
  update_participant_entities(const rmw_dds_common::msg::ParticipantEntitiesInfo & msg);

  /**
   * @}
   * \defgroup local_api
   * Methods used to update the Graph Cache, based on local construction and destruction of objects.
   * @{
   */

  RMW_DDS_COMMON_PUBLIC
  void
  add_participant(const rmw_gid_t & participant_gid);

  /**
   * Add a node to the graph, and get the message to be sent.
   *
   * /param gid participant GUID.
   * /param node_name name of the node to be added.
   * /param node_namespace node namespace.
   * /return message to be sent.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo
  add_node(
    const rmw_gid_t & participant_gid,
    const std::string & node_name,
    const std::string & node_namespace);

  /**
   * Remove a node to the graph, and get the message to be sent.
   *
   * /param gid participant GUID.
   * /param node_name name of the node to be added.
   * /param node_namespace node namespace.
   * /return message to be sent.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo
  remove_node(
    const rmw_gid_t & participant_gid,
    const std::string & node_name,
    const std::string & node_namespace);

  /**
   * Associate a writer with a node, and get a message to be sent.
   *
   * /param writer_gid GUID of the data writer.
   * /param participant_gid participant GUID.
   * /param node_name name of the node to be added.
   * /param node_namespace node namespace.
   * /return message to be sent.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo
  associate_writer(
    const rmw_gid_t & writer_gid,
    const rmw_gid_t & participant_gid,
    const std::string & node_name,
    const std::string & node_namespace);

  /**
   * Dissociate a writer with a node, and get a message to be sent.
   *
   * /param writer_gid GUID of the data writer.
   * /param participant_gid participant GUID.
   * /param node_name name of the node to be added.
   * /param node_namespace node namespace.
   * /return message to be sent.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo
  dissociate_writer(
    const rmw_gid_t & writer_gid,
    const rmw_gid_t & participant_gid,
    const std::string & node_name,
    const std::string & node_namespace);

  /**
   * Associate a reader with a node, and get a message to be sent.
   *
   * /param reader_gid GUID of the data reader.
   * /param participant_gid participant GUID.
   * /param node_name name of the node to be added.
   * /param node_namespace node namespace.
   * /return message to be sent.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo
  associate_reader(
    const rmw_gid_t & reader_gid,
    const rmw_gid_t & participant_gid,
    const std::string & node_name,
    const std::string & node_namespace);

  /**
   * Dissociate a reader with a node, and get a message to be sent.
   *
   * /param reader_gid GUID of the data reader.
   * /param participant_gid participant GUID.
   * /param node_name name of the node to be added.
   * /param node_namespace node namespace.
   * /return message to be sent.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_dds_common::msg::ParticipantEntitiesInfo
  dissociate_reader(
    const rmw_gid_t & reader_gid,
    const rmw_gid_t & participant_gid,
    const std::string & node_name,
    const std::string & node_namespace);

  /**
   * @}
   * \defgroup introspection_api
   * Methods used to introspect the GraphCache.
   * @{
   */

  /**
   * Get the number of publishers of a topic.
   * 
   * \param[in] topic_name Name of the topic.
   * \param[out] count The result will be populated there.
   *
   * \return RMW_RET_INVALID_ARGUMENT, or
   * \return RMW_RET_ERROR, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_writer_count(
    const std::string & topic_name,
    size_t * count) const;

  /**
   * Get the number of subscriptions of a topic.
   *
   * \param[in] topic_name Name of the topic.
   * \param[out] count The result will be populated there.
   *
   * \return RMW_RET_INVALID_ARGUMENT, or
   * \return RMW_RET_ERROR, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_reader_count(
    const std::string & topic_name,
    size_t * count) const;

  using DemangleFunctionT = std::string (*)(const std::string &);

  /**
   * Get all the topic names and types.
   *
   * \param[in] demangle_topic Function that indicates how a dds topic name is demangled
   *   into a ros topic name.
   * \param[in] demangle_type Function that indicates how a dds type name is demangled
   *   into a ros type name.
   * \param[in] allocator.
   * \param[inout] topic_names_and_types A zero initialized names and types object, that
   *   will be populated with the result.
   *
   * \return RMW_RET_NODE_NAME_NON_EXISTENT if the node doesn't exist, or
   * \return RMW_RET_INVALID_ARGUMENT if an argument is invalid, or
   * \return RMW_RET_BAD_ALLOC if an allocation failed, or
   * \return RMW_RET_ERROR if an unexpected error happened, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_names_and_types(
    DemangleFunctionT demangle_topic,
    DemangleFunctionT demangle_type,
    rcutils_allocator_t * allocator,
    rmw_names_and_types_t * topic_names_and_types) const;

  /**
   * Get the topic names and types that a node is publishing.
   *
   * \param[in] node_name Name of the node.
   * \param[in] node_namespace Namespace of the node.
   * \param[in] demangle_topic Function that indicates how a dds topic name is demangled
   *   into a ros topic name.
   * \param[in] demangle_type Function that indicates how a dds type name is demangled
   *   into a ros type name.
   * \param[in] allocator.
   * \param[inout] topic_names_and_types A zero initialized names and types object, that
   *   will be populated with the result.
   *
   * \return RMW_RET_NODE_NAME_NON_EXISTENT if the node doesn't exist, or
   * \return RMW_RET_INVALID_ARGUMENT if an argument is invalid, or
   * \return RMW_RET_BAD_ALLOC if an allocation failed, or
   * \return RMW_RET_ERROR if an unexpected error happened, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_writer_names_and_types_by_node(
    const std::string & node_name,
    const std::string & namespace_,
    DemangleFunctionT demangle_topic,
    DemangleFunctionT demangle_type,
    rcutils_allocator_t * allocator,
    rmw_names_and_types_t * topic_names_and_types) const;

  /**
   * Get the topic names and types that a node is subscribing.
   *
   * \param[in] node_name Name of the node.
   * \param[in] node_namespace Namespace of the node.
   * \param[in] demangle_topic Function that indicates how a dds topic name is demangled
   *   into a ros topic name.
   * \param[in] demangle_type Function that indicates how a dds type name is demangled
   *   into a ros type name.
   * \param[in] allocator.
   * \param[inout] topic_names_and_types A zero initialized names and types object, that
   *   will be populated with the result.
   *
   * \return RMW_RET_NODE_NAME_NON_EXISTENT if the node doesn't exist, or
   * \return RMW_RET_INVALID_ARGUMENT if an argument is invalid, or
   * \return RMW_RET_BAD_ALLOC if an allocation failed, or
   * \return RMW_RET_ERROR if an unexpected error happened, or
   * \return RMW_RET_OK.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_reader_names_and_types_by_node(
    const std::string & node_name,
    const std::string & namespace_,
    DemangleFunctionT demangle_topic,
    DemangleFunctionT demangle_type,
    rcutils_allocator_t * allocator,
    rmw_names_and_types_t * topic_names_and_types) const;

  /**
   * Get the number of nodes that have been discovered.
   *
   * \return RMW_RET_OK, or
   * \return RMW_RET_ERROR.
   */
  RMW_DDS_COMMON_PUBLIC
  size_t
  get_number_of_nodes() const;

  /**
   * Copy the names and namespaces of the discovered nodes.
   *
   * \param[inout] node_names A zero initialized string array, where the node names will be copied.
   * \param[inout] node_namespaces A zero initialized string array, where the node namespaces
   *   will be copied. Each item in this array corresponds to an item in the same position of
   *   node_names array.
   * \param[in] allocator.
   * \return RMW_RET_OK, or
   * \return RMW_RET_INVALID_ARGUMENT, or
   * \return RMW_RET_BAD_ALLOC, or
   * \return RMW_RET_ERROR.
   */
  RMW_DDS_COMMON_PUBLIC
  rmw_ret_t
  get_node_names(
    rcutils_string_array_t * node_names,
    rcutils_string_array_t * node_namespaces,
    rcutils_allocator_t * allocator) const;

  /**
   * @}
   */

  using NodeEntitiesInfoSeq =
    decltype(std::declval<rmw_dds_common::msg::ParticipantEntitiesInfo>().node_entities_info_seq);
  using NamespaceNamePair = std::pair<std::string, std::string>;
  using EntityGidToInfo = std::map<rmw_gid_t, EntityInfo, Compare_rmw_gid_t>;
  using ParticipantToNodesMap = std::map<rmw_gid_t, NodeEntitiesInfoSeq, Compare_rmw_gid_t>;
  using GidSeq =
    decltype(std::declval<rmw_dds_common::msg::NodeEntitiesInfo>().writer_gid_seq);

private:
  EntityGidToInfo data_writers_;
  EntityGidToInfo data_readers_;
  ParticipantToNodesMap participants_;

  mutable std::mutex mutex_;
};

RMW_DDS_COMMON_PUBLIC
std::ostream &
operator<<(std::ostream & ostream, const GraphCache & topic_cache);

class StringPairHash
{
public:
  // Based on boost::hash_combine. See
  // https://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine.
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

struct EntityInfo
{
  std::string topic_name;
  std::string topic_type;

  EntityInfo(std::string topic_name, std::string topic_type)
  : topic_name(topic_name),
    topic_type(topic_type)
  {}
};

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__GRAPH_CACHE_HPP_
