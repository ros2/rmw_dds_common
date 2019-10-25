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

#include <algorithm>
#include <cassert>
#include <iterator>
#include <mutex>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rcutils/strdup.h"

#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/error_handling.h"
#include "rmw/sanity_checks.h"

#include "rmw_dds_common/graph_cache.hpp"
#include "rmw_dds_common/gid_utils.hpp"

using rmw_dds_common::GraphCache;
using rmw_dds_common::operator<<;

const char log_tag[] = "rmw_dds_common";

bool
GraphCache::add_writer(
  const rmw_gid_t & gid,
  const std::string & topic_name,
  const std::string & type_name)
{
  std::lock_guard<std::mutex> guard(mutex_);
  auto pair = data_writers_.emplace(
    std::piecewise_construct,
    std::forward_as_tuple(gid),
    std::forward_as_tuple(topic_name, type_name));
  return pair.second;
}

bool
GraphCache::add_reader(
  const rmw_gid_t & gid,
  const std::string & topic_name,
  const std::string & type_name)
{
  std::lock_guard<std::mutex> guard(mutex_);
  auto pair = data_readers_.emplace(
    std::piecewise_construct,
    std::forward_as_tuple(gid),
    std::forward_as_tuple(topic_name, type_name));
  return pair.second;
}

bool
GraphCache::add_entity(
  const rmw_gid_t & gid,
  const std::string & topic_name,
  const std::string & type_name,
  bool is_reader)
{
  if (is_reader) {
    // fprintf(stderr, "added reader %s\n", topic_name.c_str());
    return this->add_reader(
      gid,
      topic_name,
      type_name);
  }
  // fprintf(stderr, "added writer %s\n", topic_name.c_str());
  return this->add_writer(
    gid,
    topic_name,
    type_name);
}

bool
GraphCache::remove_writer(const rmw_gid_t & gid)
{
  return data_writers_.erase(gid) > 0;
}

bool
GraphCache::remove_reader(const rmw_gid_t & gid)
{
  return data_readers_.erase(gid) > 0;
}

bool
GraphCache::remove_entity(const rmw_gid_t & gid, bool is_reader)
{
  if (is_reader) {
    return this->remove_reader(gid);
  }
  return this->remove_writer(gid);
}

bool
GraphCache::update_participant_entities(rmw_dds_common::msg::ParticipantEntitiesInfo & msg)
{
  rmw_gid_t gid;
  rmw_dds_common::convert_msg_to_gid(&msg.gid, &gid);
  auto pair = participants_.emplace(
    std::piecewise_construct,
    std::forward_as_tuple(gid),
    std::forward_as_tuple(msg.node_entities_info_seq));
  return pair.second;
}

bool
GraphCache::remove_participant(const rmw_gid_t & participant_gid)
{
  return participants_.erase(participant_gid) > 0;
}

static
rmw_dds_common::msg::ParticipantEntitiesInfo
__create_participant_info_message(
  const rmw_gid_t & gid,
  const GraphCache::NodeEntitiesInfoSeq & info)
{
  rmw_dds_common::msg::ParticipantEntitiesInfo msg;
  rmw_dds_common::convert_gid_to_msg(&gid, &msg.gid);
  msg.node_entities_info_seq = info;
  return msg;
}

void
GraphCache::add_participant(const rmw_gid_t & participant_gid)
{
  participants_.emplace(
    std::piecewise_construct,
    std::forward_as_tuple(participant_gid),
    std::forward_as_tuple());
}

rmw_dds_common::msg::ParticipantEntitiesInfo
GraphCache::add_node(
  const rmw_gid_t & participant_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  auto it = participants_.find(participant_gid);
  assert(it != participants_.end());

  // TODO(ivanpauno): We could check local name duplication here, and return an error in that case.
  // Consider that in the node name uniqueness discussion.
  rmw_dds_common::msg::NodeEntitiesInfo node_info;
  node_info.node_name = node_name;
  node_info.node_namespace = node_namespace;
  it->second.emplace_back(node_info);

  return __create_participant_info_message(participant_gid, it->second);
}

rmw_dds_common::msg::ParticipantEntitiesInfo
GraphCache::remove_node(
  const rmw_gid_t & participant_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  auto it = participants_.find(participant_gid);
  assert(it != participants_.end());

  auto new_end = std::remove_if(
    it->second.begin(),
    it->second.end(),
    [&](const rmw_dds_common::msg::NodeEntitiesInfo & node_info) {
      return node_info.node_name == node_name && node_info.node_namespace == node_namespace;
    });

  assert(new_end != it->second.end());

  it->second.erase(new_end, it->second.end());

  return __create_participant_info_message(participant_gid, it->second);
}

template<typename FunctorT>
rmw_dds_common::msg::ParticipantEntitiesInfo
__modify_node_info(
  const rmw_gid_t & participant_gid,
  const std::string & node_name,
  const std::string & node_namespace,
  FunctorT do_sth,
  GraphCache::ParticipantToNodesMap & participant_map)
{
  auto participant_info = participant_map.find(participant_gid);
  assert(participant_info != participant_map.end());
  auto node_info = std::find_if(
    participant_info->second.begin(),
    participant_info->second.end(),
    [&](const rmw_dds_common::msg::NodeEntitiesInfo & node_info)
    {
      return node_info.node_name == node_name && node_info.node_namespace == node_namespace;
    });
  assert(node_info != participant_info->second.end());

  do_sth(*node_info);
  return __create_participant_info_message(participant_gid, participant_info->second);
}

rmw_dds_common::msg::ParticipantEntitiesInfo
GraphCache::associate_writer(
  const rmw_gid_t & writer_gid,
  const rmw_gid_t & participant_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  auto add_writer_gid = [&](rmw_dds_common::msg::NodeEntitiesInfo & info)
    {
      info.writer_gid_seq.emplace_back();
      convert_gid_to_msg(&writer_gid, &info.writer_gid_seq.back());
    };

  return __modify_node_info(
    participant_gid, node_name, node_namespace, add_writer_gid, participants_);
}

rmw_dds_common::msg::ParticipantEntitiesInfo
GraphCache::deassociate_writer(
  const rmw_gid_t & writer_gid,
  const rmw_gid_t & participant_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  rmw_dds_common::msg::Gid writer_gid_msg;
  convert_gid_to_msg(&writer_gid, &writer_gid_msg);
  auto delete_writer_gid = [&](rmw_dds_common::msg::NodeEntitiesInfo & info)
    {
      info.writer_gid_seq.erase(
        std::find_if(
          info.writer_gid_seq.begin(),
          info.writer_gid_seq.end(),
          [&](const rmw_dds_common::msg::Gid & gid)
          {
            return gid == writer_gid_msg;
          }));
    };

  return __modify_node_info(
    participant_gid, node_name, node_namespace, delete_writer_gid, participants_);
}

rmw_dds_common::msg::ParticipantEntitiesInfo
GraphCache::associate_reader(
  const rmw_gid_t & reader_gid,
  const rmw_gid_t & participant_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  auto add_reader_gid = [&](rmw_dds_common::msg::NodeEntitiesInfo & info)
    {
      info.reader_gid_seq.emplace_back();
      convert_gid_to_msg(&reader_gid, &info.reader_gid_seq.back());
    };

  return __modify_node_info(
    participant_gid, node_name, node_namespace, add_reader_gid, participants_);
}

rmw_dds_common::msg::ParticipantEntitiesInfo
GraphCache::deassociate_reader(
  const rmw_gid_t & reader_gid,
  const rmw_gid_t & participant_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  rmw_dds_common::msg::Gid reader_gid_msg;
  convert_gid_to_msg(&reader_gid, &reader_gid_msg);
  auto delete_reader_gid = [&](rmw_dds_common::msg::NodeEntitiesInfo & info)
    {
      info.reader_gid_seq.erase(
        std::find_if(
          info.reader_gid_seq.begin(),
          info.reader_gid_seq.end(),
          [&](const rmw_dds_common::msg::Gid & gid)
          {
            return gid == reader_gid_msg;
          }));
    };

  return __modify_node_info(
    participant_gid, node_name, node_namespace, delete_reader_gid, participants_);
}

static
rmw_ret_t
__get_count(
  const GraphCache::EntityGidToInfo & entities,
  std::string topic_name,
  size_t * count)
{
  assert(nullptr != count);

  *count = std::count_if(
    entities.begin(),
    entities.end(),
    [&topic_name](const GraphCache::EntityGidToInfo::value_type & elem)
    {
      return elem.second.topic_name == topic_name;
    });
  return RMW_RET_OK;
}

rmw_ret_t
GraphCache::get_writer_count(
  const std::string & topic_name,
  size_t * count) const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return __get_count(data_writers_, topic_name, count);
}

rmw_ret_t
GraphCache::get_reader_count(
  const std::string & topic_name,
  size_t * count) const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return __get_count(data_readers_, topic_name, count);
}

using NamesAndTypes = std::unordered_map<std::string, std::set<std::string>>;

static
void
__get_names_and_types(
  const GraphCache::EntityGidToInfo & entities,
  std::string (* demangle_topic)(const std::string &),
  NamesAndTypes & topics)
{
  for (const auto & item : entities) {
    std::string demangled_topic_name = demangle_topic(item.second.topic_name);
    if ("" != demangled_topic_name) {
      topics[demangled_topic_name].insert(item.second.topic_type);
    }
  }
}

static
rmw_ret_t
__copy_data_to_results(
  NamesAndTypes topics,
  std::string (* demangle_type)(const std::string &),
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * topic_names_and_types)
{
  if (topics.empty()) {
    return RMW_RET_OK;
  }

  rmw_ret_t rmw_ret = rmw_names_and_types_init(topic_names_and_types, topics.size(), allocator);
  if (RMW_RET_OK != rmw_ret) {
    return rmw_ret;
  }

  size_t index = 0;
  for (const auto & item : topics) {
    char * topic_name = rcutils_strdup(item.first.c_str(), *allocator);
    if (!topic_name) {
      RMW_SET_ERROR_MSG("failed to allocate memory for topic name");
      rmw_ret = RMW_RET_BAD_ALLOC;
      goto cleanup;
    }
    topic_names_and_types->names.data[index] = topic_name;

    {
      rcutils_ret_t rcutils_ret = rcutils_string_array_init(
        &topic_names_and_types->types[index],
        item.second.size(),
        allocator);
      if (rcutils_ret != RCUTILS_RET_OK) {
        RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
        rmw_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
        goto cleanup;
      }
    }
    size_t type_index = 0;
    for (const auto & type : item.second) {
      char * type_name = rcutils_strdup(demangle_type(type).c_str(), *allocator);
      if (!type_name) {
        RMW_SET_ERROR_MSG("failed to allocate memory for type name");
        rmw_ret = RMW_RET_BAD_ALLOC;
        goto cleanup;
      }
      topic_names_and_types->types[index].data[type_index] = type_name;
      ++type_index;
    }
    ++index;
  }
  return RMW_RET_OK;
cleanup:
  if (RMW_RET_OK != rmw_names_and_types_fini(topic_names_and_types)) {
    RCUTILS_LOG_ERROR_NAMED(
      log_tag,
      "error during report of error: %s", rmw_get_error_string().str);
  }
  return rmw_ret;
}

rmw_ret_t
GraphCache::get_names_and_types(
  std::string (* demangle_topic)(const std::string &),
  std::string (* demangle_type)(const std::string &),
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * topic_names_and_types) const
{
  assert(demangle_topic);
  assert(demangle_type);
  assert(allocator);
  assert(topic_names_and_types);

  NamesAndTypes topics;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    __get_names_and_types(
      data_readers_,
      demangle_topic,
      topics);
    __get_names_and_types(
      data_writers_,
      demangle_topic,
      topics);
  }

  return __copy_data_to_results(
    topics,
    demangle_type,
    allocator,
    topic_names_and_types);
}

static
const rmw_dds_common::msg::NodeEntitiesInfo *
__find_node(
  const GraphCache::ParticipantToNodesMap & participant_map,
  const std::string & node_name,
  const std::string & node_namespace)
{
  for (const auto & participant : participant_map) {
    for (const auto & node : participant.second) {
      if (
        node.node_name == node_name &&
        node.node_namespace == node_namespace)
      {
        return &node;
      }
    }
  }
  return nullptr;
}

static
NamesAndTypes
__get_names_and_types_from_gids(
  const GraphCache::EntityGidToInfo & entities_map,
  const GraphCache::GidSeq & gids,
  std::string (* demangle_topic)(const std::string &))
{
  NamesAndTypes topics;

  for (const auto & gid_msg : gids) {
    rmw_gid_t gid;
    rmw_dds_common::convert_msg_to_gid(&gid_msg, &gid);
    auto it = entities_map.find(gid);
    if (it == entities_map.end()) {
      continue;
    }
    std::string demangled_topic_name = demangle_topic(it->second.topic_name);
    if ("" == demangled_topic_name) {
      continue;
    }
    topics[demangled_topic_name].insert(it->second.topic_type);
  }
  return topics;
}

static
rmw_ret_t
__get_names_and_types_by_node(
  const GraphCache::ParticipantToNodesMap & participants_map,
  const GraphCache::EntityGidToInfo & entities_map,
  const std::string & node_name,
  const std::string & namespace_,
  std::string (* demangle_topic)(const std::string &),
  std::string (* demangle_type)(const std::string &),
  GraphCache::GidSeq (* get_entities_gids)(const rmw_dds_common::msg::NodeEntitiesInfo &),
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * topic_names_and_types)
{
  assert(demangle_topic);
  assert(demangle_type);
  assert(allocator);
  assert(topic_names_and_types);

  auto node_info_ptr = __find_node(
    participants_map,
    node_name,
    namespace_);

  if (nullptr == node_info_ptr) {
    return RMW_RET_NODE_NAME_NON_EXISTENT;
  }

  NamesAndTypes topics = __get_names_and_types_from_gids(
    entities_map,
    get_entities_gids(*node_info_ptr),
    demangle_topic);

  return __copy_data_to_results(
    topics,
    demangle_type,
    allocator,
    topic_names_and_types);
}

static
GraphCache::GidSeq
__get_writers_gids(const rmw_dds_common::msg::NodeEntitiesInfo & node_info)
{
  return node_info.writer_gid_seq;
}

rmw_ret_t
GraphCache::get_writer_names_and_types_by_node(
  const std::string & node_name,
  const std::string & namespace_,
  std::string (* demangle_topic)(const std::string &),
  std::string (* demangle_type)(const std::string &),
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * topic_names_and_types) const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return __get_names_and_types_by_node(
    participants_,
    data_writers_,
    node_name,
    namespace_,
    demangle_topic,
    demangle_type,
    __get_writers_gids,
    allocator,
    topic_names_and_types);
}

static
GraphCache::GidSeq
__get_readers_gids(const rmw_dds_common::msg::NodeEntitiesInfo & node_info)
{
  return node_info.reader_gid_seq;
}

rmw_ret_t
GraphCache::get_reader_names_and_types_by_node(
  const std::string & node_name,
  const std::string & namespace_,
  std::string (* demangle_topic)(const std::string &),
  std::string (* demangle_type)(const std::string &),
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * topic_names_and_types) const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return __get_names_and_types_by_node(
    participants_,
    data_readers_,
    node_name,
    namespace_,
    demangle_topic,
    demangle_type,
    __get_readers_gids,
    allocator,
    topic_names_and_types);
}

size_t
GraphCache::get_number_of_nodes() const
{
  size_t nodes_number = 0;
  for (const auto & elem : participants_) {
    nodes_number += elem.second.size();
  }
  return nodes_number;
}

rmw_ret_t
GraphCache::get_node_names(
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

  size_t nodes_number = this->get_number_of_nodes();
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
    for (const auto & elem : participants_) {
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

std::ostream &
rmw_dds_common::operator<<(std::ostream & ostream, const GraphCache & graph_cache)
{
  (void)graph_cache;
  // std::lock_guard<std::mutex> guard(topic_cache.mutex_);
  return ostream;

  // TODO(ivanpauno): Reimplement this
  // std::ostringstream ss;
  // ss << "Participant Info: " << std::endl;
  // for (const auto & gid_node_map_pair : topic_cache.get_participant_to_nodes_to_topics()) {
  //   ss << "  gid: " << gid_node_map_pair.first << std::endl;
  //   for (const auto & node_topic_map_pair : gid_node_map_pair.second) {
  //     ss << "    Node:" << std::endl;
  //     ss << "      ns='" << node_topic_map_pair.first.first << "'" << std::endl;
  //     ss << "      name='" << node_topic_map_pair.first.second << "'" << std::endl;
  //     ss << "    Node:" << std::endl;
  //     ss << "      Topics: " << std::endl;
  //     for (const auto & topic_types_pair : node_topic_map_pair.second) {
  //       ss << "        " << topic_types_pair.first << ": ";
  //       std::copy(topic_types_pair.second.begin(), topic_types_pair.second.end(),
  //         std::ostream_iterator<std::string>(ostream, ","));
  //       ss << std::endl;
  //     }
  //   }
  // }
  // ss << "Cumulative TopicToTypes: " << std::endl;
  // for (auto & elem : topic_cache.get_topic_to_types()) {
  //   ss << "  " << elem.first << ": ";
  //   std::copy(elem.second.begin(), elem.second.end(), std::ostream_iterator<std::string>(ostream,
  //     ","));
  //   ss << std::endl;
  // }
  // return ss << ss.str();
}
