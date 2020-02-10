// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "rmw_dds_common/graph_cache.hpp"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rcutils/strdup.h"

#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/error_handling.h"
#include "rmw/sanity_checks.h"
#include "rmw/topic_endpoint_info.h"
#include "rmw/topic_endpoint_info_array.h"

#include "rmw_dds_common/gid_utils.hpp"

using rmw_dds_common::GraphCache;
using rmw_dds_common::operator<<;

static const char log_tag[] = "rmw_dds_common";

bool
GraphCache::add_writer(
  const rmw_gid_t & gid,
  const std::string & topic_name,
  const std::string & type_name,
  const rmw_gid_t & participant_gid,
  const rmw_qos_profile_t & qos)
{
  std::lock_guard<std::mutex> guard(mutex_);
  auto pair = data_writers_.emplace(
    std::piecewise_construct,
    std::forward_as_tuple(gid),
    std::forward_as_tuple(topic_name, type_name, participant_gid, qos));
  return pair.second;
}

bool
GraphCache::add_reader(
  const rmw_gid_t & gid,
  const std::string & topic_name,
  const std::string & type_name,
  const rmw_gid_t & participant_gid,
  const rmw_qos_profile_t & qos)
{
  std::lock_guard<std::mutex> guard(mutex_);
  auto pair = data_readers_.emplace(
    std::piecewise_construct,
    std::forward_as_tuple(gid),
    std::forward_as_tuple(topic_name, type_name, participant_gid, qos));
  return pair.second;
}

bool
GraphCache::add_entity(
  const rmw_gid_t & gid,
  const std::string & topic_name,
  const std::string & type_name,
  const rmw_gid_t & participant_gid,
  const rmw_qos_profile_t & qos,
  bool is_reader)
{
  if (is_reader) {
    return this->add_reader(
      gid,
      topic_name,
      type_name,
      participant_gid,
      qos);
  }
  return this->add_writer(
    gid,
    topic_name,
    type_name,
    participant_gid,
    qos);
}

bool
GraphCache::remove_writer(const rmw_gid_t & gid)
{
  std::lock_guard<std::mutex> guard(mutex_);
  return data_writers_.erase(gid) > 0;
}

bool
GraphCache::remove_reader(const rmw_gid_t & gid)
{
  std::lock_guard<std::mutex> guard(mutex_);
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

void
GraphCache::update_participant_entities(const rmw_dds_common::msg::ParticipantEntitiesInfo & msg)
{
  std::lock_guard<std::mutex> guard(mutex_);
  rmw_gid_t gid;
  rmw_dds_common::convert_msg_to_gid(&msg.gid, &gid);
  participants_[gid] = msg.node_entities_info_seq;
}

bool
GraphCache::remove_participant(const rmw_gid_t & participant_gid)
{
  std::lock_guard<std::mutex> guard(mutex_);
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
  std::lock_guard<std::mutex> guard(mutex_);
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
  std::lock_guard<std::mutex> guard(mutex_);
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
  std::lock_guard<std::mutex> guard(mutex_);
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
  FunctorT func,
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

  func(*node_info);
  return __create_participant_info_message(participant_gid, participant_info->second);
}

rmw_dds_common::msg::ParticipantEntitiesInfo
GraphCache::associate_writer(
  const rmw_gid_t & writer_gid,
  const rmw_gid_t & participant_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  std::lock_guard<std::mutex> guard(mutex_);
  auto add_writer_gid = [&](rmw_dds_common::msg::NodeEntitiesInfo & info)
    {
      info.writer_gid_seq.emplace_back();
      convert_gid_to_msg(&writer_gid, &info.writer_gid_seq.back());
    };

  return __modify_node_info(
    participant_gid, node_name, node_namespace, add_writer_gid, participants_);
}

rmw_dds_common::msg::ParticipantEntitiesInfo
GraphCache::dissociate_writer(
  const rmw_gid_t & writer_gid,
  const rmw_gid_t & participant_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  std::lock_guard<std::mutex> guard(mutex_);
  rmw_dds_common::msg::Gid writer_gid_msg;
  convert_gid_to_msg(&writer_gid, &writer_gid_msg);
  auto delete_writer_gid = [&](rmw_dds_common::msg::NodeEntitiesInfo & info)
    {
      auto it = std::find_if(
        info.writer_gid_seq.begin(),
        info.writer_gid_seq.end(),
        [&](const rmw_dds_common::msg::Gid & gid)
        {
          return gid == writer_gid_msg;
        });
      if (it != info.writer_gid_seq.end()) {
        info.writer_gid_seq.erase(it);
      }
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
  std::lock_guard<std::mutex> guard(mutex_);
  auto add_reader_gid = [&reader_gid](rmw_dds_common::msg::NodeEntitiesInfo & info)
    {
      info.reader_gid_seq.emplace_back();
      convert_gid_to_msg(&reader_gid, &info.reader_gid_seq.back());
    };

  return __modify_node_info(
    participant_gid, node_name, node_namespace, add_reader_gid, participants_);
}

rmw_dds_common::msg::ParticipantEntitiesInfo
GraphCache::dissociate_reader(
  const rmw_gid_t & reader_gid,
  const rmw_gid_t & participant_gid,
  const std::string & node_name,
  const std::string & node_namespace)
{
  std::lock_guard<std::mutex> guard(mutex_);
  rmw_dds_common::msg::Gid reader_gid_msg;
  convert_gid_to_msg(&reader_gid, &reader_gid_msg);
  auto delete_reader_gid = [&](rmw_dds_common::msg::NodeEntitiesInfo & info)
    {
      auto it = std::find_if(
        info.reader_gid_seq.begin(),
        info.reader_gid_seq.end(),
        [&](const rmw_dds_common::msg::Gid & gid)
        {
          return gid == reader_gid_msg;
        });
      if (it != info.reader_gid_seq.end()) {
        info.reader_gid_seq.erase(it);
      }
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
  assert(count);

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
  if (!count) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  return __get_count(data_writers_, topic_name, count);
}

rmw_ret_t
GraphCache::get_reader_count(
  const std::string & topic_name,
  size_t * count) const
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (!count) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  return __get_count(data_readers_, topic_name, count);
}

static
std::tuple<std::string, std::string, bool>
__find_name_and_namespace_from_entity_gid(
  const GraphCache::ParticipantToNodesMap & participant_map,
  rmw_gid_t participant_gid,
  rmw_gid_t entity_gid,
  bool is_reader)
{
  auto it = participant_map.find(participant_gid);
  if (participant_map.end() == it) {
    return {"", "", false};
  }
  for (const auto & node_info : it->second) {
    auto & gid_seq = is_reader ? node_info.reader_gid_seq : node_info.writer_gid_seq;
    auto it = std::find_if(
      gid_seq.begin(),
      gid_seq.end(),
      [&](const rmw_dds_common::msg::Gid & gid) -> bool {
        return 0u == std::memcmp(gid.data.data(), entity_gid.data, RMW_GID_STORAGE_SIZE);
      }
    );
    if (gid_seq.end() != it) {
      return {node_info.node_name, node_info.node_namespace, true};
    }
  }
  return {"", "", false};
}

using DemangleFunctionT = GraphCache::DemangleFunctionT;

static
rmw_ret_t
__get_entities_info_by_topic(
  const GraphCache::EntityGidToInfo & entities,
  const GraphCache::ParticipantToNodesMap & participant_map,
  const std::string & topic_name,
  DemangleFunctionT demangle_type,
  bool is_reader,
  rcutils_allocator_t * allocator,
  rmw_topic_endpoint_info_array_t * endpoints_info)
{
  assert(allocator);
  assert(endpoints_info);

  if (0u == entities.size()) {
    return RMW_RET_OK;
  }

  size_t size = std::count_if(
    entities.begin(),
    entities.end(),
    [&topic_name](const GraphCache::EntityGidToInfo::value_type & item) {
      return item.second.topic_name == topic_name;
    }
  );
  if (0u == size) {
    return RMW_RET_OK;
  }

  rmw_ret_t ret = rmw_topic_endpoint_info_array_init_with_size(
    endpoints_info,
    size,
    allocator);
  // TODO(ivanpauno): `count` should be initialized by
  //  `rmw_topic_endpoint_info_array_init_with_size`, and should be probably called `size`.
  endpoints_info->count = size;
  if (RMW_RET_OK != ret) {
    return ret;
  }
  std::unique_ptr<
    rmw_topic_endpoint_info_array_t,
    std::function<void(rmw_topic_endpoint_info_array_t *)>>
  endpoints_info_delete_on_error(
    endpoints_info,
    [allocator](rmw_topic_endpoint_info_array_t * p) {
      rmw_ret_t ret = rmw_topic_endpoint_info_array_fini(
        p,
        allocator
      );
      if (RMW_RET_OK != ret) {
        RCUTILS_LOG_ERROR_NAMED(
          log_tag,
          "Failed to destroy endpoints_info when function failed.");
      }
    }
  );

  size_t i = 0;
  for (const auto & entity_pair : entities) {
    if (entity_pair.second.topic_name != topic_name) {
      continue;
    }

    rmw_topic_endpoint_info_t & endpoint_info = endpoints_info->info_array[i];
    endpoint_info = rmw_get_zero_initialized_topic_endpoint_info();

    auto result = __find_name_and_namespace_from_entity_gid(
      participant_map,
      entity_pair.second.participant_gid,
      entity_pair.first,
      is_reader);

    std::string node_name;
    std::string node_namespace;
    if (std::get<2>(result)) {
      node_name = std::move(std::get<0>(result));
      node_namespace = std::move(std::get<1>(result));
    } else {
      node_name = "_NODE_NAME_UNKNOWN_";
      node_namespace = "_NODE_NAMESPACE_UNKNOWN_";
    }

    ret = rmw_topic_endpoint_info_set_node_name(
      &endpoint_info,
      node_name.c_str(),
      allocator);
    if (RMW_RET_OK != ret) {
      return ret;
    }

    ret = rmw_topic_endpoint_info_set_node_namespace(
      &endpoint_info,
      node_namespace.c_str(),
      allocator);
    if (RMW_RET_OK != ret) {
      return ret;
    }

    ret = rmw_topic_endpoint_info_set_topic_type(
      &endpoint_info,
      demangle_type(entity_pair.second.topic_type).c_str(),
      allocator);
    if (RMW_RET_OK != ret) {
      return ret;
    }

    ret = rmw_topic_endpoint_info_set_endpoint_type(
      &endpoint_info,
      is_reader ? RMW_ENDPOINT_SUBSCRIPTION : RMW_ENDPOINT_PUBLISHER);
    if (RMW_RET_OK != ret) {
      return ret;
    }

    ret = rmw_topic_endpoint_info_set_gid(
      &endpoint_info,
      entity_pair.first.data,
      RMW_GID_STORAGE_SIZE);
    if (RMW_RET_OK != ret) {
      return ret;
    }

    ret = rmw_topic_endpoint_info_set_qos_profile(
      &endpoint_info,
      &entity_pair.second.qos);
    if (RMW_RET_OK != ret) {
      return ret;
    }
    i++;
  }

  endpoints_info_delete_on_error.release();
  return RMW_RET_OK;
}

rmw_ret_t
GraphCache::get_writers_info_by_topic(
  const std::string & topic_name,
  DemangleFunctionT demangle_type,
  rcutils_allocator_t * allocator,
  rmw_topic_endpoint_info_array_t * endpoints_info) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return __get_entities_info_by_topic(
    data_writers_,
    participants_,
    topic_name,
    demangle_type,
    false,
    allocator,
    endpoints_info);
}

rmw_ret_t
GraphCache::get_readers_info_by_topic(
  const std::string & topic_name,
  DemangleFunctionT demangle_type,
  rcutils_allocator_t * allocator,
  rmw_topic_endpoint_info_array_t * endpoints_info) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return __get_entities_info_by_topic(
    data_readers_,
    participants_,
    topic_name,
    demangle_type,
    true,
    allocator,
    endpoints_info);
}

using NamesAndTypes = std::map<std::string, std::set<std::string>>;

static
void
__get_names_and_types(
  const GraphCache::EntityGidToInfo & entities,
  DemangleFunctionT demangle_topic,
  DemangleFunctionT demangle_type,
  NamesAndTypes & topics)
{
  assert(nullptr != demangle_topic);
  assert(nullptr != demangle_type);
  for (const auto & item : entities) {
    std::string demangled_topic_name = demangle_topic(item.second.topic_name);
    if ("" != demangled_topic_name) {
      topics[demangled_topic_name].insert(demangle_type(item.second.topic_type));
    }
  }
}

static
rmw_ret_t
__populate_rmw_names_and_types(
  NamesAndTypes topics,
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
      if (RCUTILS_RET_OK != rcutils_ret) {
        RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
        rmw_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
        goto cleanup;
      }
    }
    size_t type_index = 0;
    for (const auto & type : item.second) {
      char * type_name = rcutils_strdup(type.c_str(), *allocator);
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
  DemangleFunctionT demangle_topic,
  DemangleFunctionT demangle_type,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * topic_names_and_types) const
{
  assert(demangle_topic);
  assert(demangle_type);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);
  if (RMW_RET_OK != rmw_names_and_types_check_zero(topic_names_and_types)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  // TODO(ivanpauno): Avoid using an intermediate representation.
  // We need a way to reallocate `topic_names_and_types.names` and `topic_names_and_types.names`.
  // Or have a good guess of the size (lower bound), and then shrink.
  NamesAndTypes topics;
  {
    std::lock_guard<std::mutex> guard(mutex_);
    __get_names_and_types(
      data_readers_,
      demangle_topic,
      demangle_type,
      topics);
    __get_names_and_types(
      data_writers_,
      demangle_topic,
      demangle_type,
      topics);
  }

  return __populate_rmw_names_and_types(
    topics,
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
  DemangleFunctionT demangle_topic,
  DemangleFunctionT demangle_type)
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
    topics[demangled_topic_name].insert(demangle_type(it->second.topic_type));
  }
  return topics;
}

using GetEntitiesGidsFuncT =
  std::function<const GraphCache::GidSeq & (const rmw_dds_common::msg::NodeEntitiesInfo &)>;

static
rmw_ret_t
__get_names_and_types_by_node(
  const GraphCache::ParticipantToNodesMap & participants_map,
  const GraphCache::EntityGidToInfo & entities_map,
  const std::string & node_name,
  const std::string & namespace_,
  DemangleFunctionT demangle_topic,
  DemangleFunctionT demangle_type,
  GetEntitiesGidsFuncT get_entities_gids,
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * topic_names_and_types)
{
  assert(demangle_topic);
  assert(demangle_type);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);
  if (RMW_RET_OK != rmw_names_and_types_check_zero(topic_names_and_types)) {
    return RMW_RET_INVALID_ARGUMENT;
  }

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
    demangle_topic,
    demangle_type);

  return __populate_rmw_names_and_types(
    topics,
    allocator,
    topic_names_and_types);
}

static
const GraphCache::GidSeq &
__get_writers_gids(const rmw_dds_common::msg::NodeEntitiesInfo & node_info)
{
  return node_info.writer_gid_seq;
}

rmw_ret_t
GraphCache::get_writer_names_and_types_by_node(
  const std::string & node_name,
  const std::string & namespace_,
  DemangleFunctionT demangle_topic,
  DemangleFunctionT demangle_type,
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
const GraphCache::GidSeq &
__get_readers_gids(const rmw_dds_common::msg::NodeEntitiesInfo & node_info)
{
  return node_info.reader_gid_seq;
}

rmw_ret_t
GraphCache::get_reader_names_and_types_by_node(
  const std::string & node_name,
  const std::string & namespace_,
  DemangleFunctionT demangle_topic,
  DemangleFunctionT demangle_type,
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

static
size_t
__get_number_of_nodes(const GraphCache::ParticipantToNodesMap & participants_map)
{
  size_t nodes_number = 0;
  for (const auto & elem : participants_map) {
    nodes_number += elem.second.size();
  }
  return nodes_number;
}

size_t
GraphCache::get_number_of_nodes() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return __get_number_of_nodes(participants_);
}

rmw_ret_t
GraphCache::get_node_names(
  rcutils_string_array_t * node_names,
  rcutils_string_array_t * node_namespaces,
  rcutils_allocator_t * allocator) const
{
  std::lock_guard<std::mutex> guard(mutex_);
  if (rmw_check_zero_rmw_string_array(node_names) != RMW_RET_OK) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (rmw_check_zero_rmw_string_array(node_namespaces) != RMW_RET_OK) {
    return RMW_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    allocator, "get_node_names allocator is not valid", return RMW_RET_INVALID_ARGUMENT);

  size_t nodes_number = __get_number_of_nodes(participants_);
  rcutils_ret_t rcutils_ret =
    rcutils_string_array_init(node_names, nodes_number, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    rcutils_reset_error();
    RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
    goto fail;
  }
  rcutils_ret =
    rcutils_string_array_init(node_namespaces, nodes_number, allocator);
  if (rcutils_ret != RCUTILS_RET_OK) {
    rcutils_reset_error();
    RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
    goto fail;
  }
  {
    size_t j = 0;
    for (const auto & elem : participants_) {
      const auto & nodes_info = elem.second;
      for (const auto & node_info : nodes_info) {
        node_names->data[j] = rcutils_strdup(node_info.node_name.c_str(), *allocator);
        if (!node_names->data[j]) {
          goto fail;
        }
        node_namespaces->data[j] = rcutils_strdup(
          node_info.node_namespace.c_str(), *allocator);
        if (!node_namespaces->data[j]) {
          goto fail;
        }
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
  }
  rcutils_ret = rcutils_string_array_fini(node_namespaces);
  if (rcutils_ret != RCUTILS_RET_OK) {
    RCUTILS_LOG_ERROR_NAMED(
      "rmw_dds_common",
      "failed to cleanup during error handling: %s", rcutils_get_error_string().str);
  }
  return RMW_RET_BAD_ALLOC;
}

std::ostream &
rmw_dds_common::operator<<(std::ostream & ostream, const GraphCache & graph_cache)
{
  std::lock_guard<std::mutex> guard(graph_cache.mutex_);
  std::ostringstream ss;

  ss << "---------------------------------" << std::endl;
  ss << "Graph cache:" << std::endl;
  ss << "  Discovered data writers:" << std::endl;
  for (const auto & data_writer_pair : graph_cache.data_writers_) {
    ss << "    gid: '" << data_writer_pair.first << "', topic name: '" <<
      data_writer_pair.second.topic_name << "', topic_type: '" <<
      data_writer_pair.second.topic_type << "'" << std::endl;
  }
  ss << "  Discovered data readers:" << std::endl;
  for (const auto & data_reader_pair : graph_cache.data_readers_) {
    ss << "    gid: '" << data_reader_pair.first << "', topic name: '" <<
      data_reader_pair.second.topic_name << "', topic_type: '" <<
      data_reader_pair.second.topic_type << "'" << std::endl;
  }
  ss << "  Discovered participants:" << std::endl;
  for (const auto & item : graph_cache.participants_) {
    ss << "    gid: '" << item.first << std::endl;
    ss << "    nodes:" << std::endl;
    for (const auto & node_info : item.second) {
      ss << "      namespace: '" << node_info.node_namespace << "' name: '" <<
        node_info.node_name << "'" << std::endl;
      ss << "      associated data readers gids:" << std::endl;
      for (const auto & data_reader_gid : node_info.reader_gid_seq) {
        rmw_gid_t rmw_gid;
        convert_msg_to_gid(&data_reader_gid, &rmw_gid);
        ss << "        " << rmw_gid << std::endl;
      }
      ss << "      associated data writers gids:" << std::endl;
      for (const auto & data_writer_gid : node_info.writer_gid_seq) {
        rmw_gid_t rmw_gid;
        convert_msg_to_gid(&data_writer_gid, &rmw_gid);
        ss << "        " << rmw_gid << std::endl;
      }
    }
  }
  ss << "---------------------------------" << std::endl;

  return ostream << ss.str();
}
