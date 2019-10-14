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
#include <sstream>
#include <string>
#include <vector>

#include "rcutils/strdup.h"
#include "rmw/convert_rcutils_ret_to_rmw_ret.h"
#include "rmw/error_handling.h"
#include "rmw_dds_common/gid_utils.hpp"
#include "rmw_dds_common/topic_cache.hpp"

using rmw_dds_common::TopicCache;
using rmw_dds_common::operator<<;

const char log_tag[] = "rmw_dds_common";

const TopicCache::TopicToTypes &
TopicCache::get_topic_to_types() const
{
  return topic_to_types_;
}

const TopicCache::ParticipantNodeMap &
TopicCache::get_participant_to_nodes_to_topics() const
{
  return participant_to_nodes_to_topics_;
}

bool
TopicCache::add_topic(
  const rmw_gid_t & gid,
  const std::string & namespace_,
  const std::string & node_name,
  const std::string & topic_name,
  const std::string & type_name)
{
  std::lock_guard<std::mutex> guard(mutex_);
  auto pair = std::make_pair<const std::string &, const std::string &>(namespace_, node_name);
  initialize_topic(topic_name, topic_to_types_);
  initialize_participant_node_map(gid, participant_to_nodes_to_topics_);
  initialize_node_topic_map(pair, participant_to_nodes_to_topics_[gid]);
  initialize_topic(topic_name, participant_to_nodes_to_topics_[gid][pair]);
  if (rcutils_logging_logger_is_enabled_for("rmw_dds_common",
    RCUTILS_LOG_SEVERITY_DEBUG))
  {
    std::stringstream gid_stream;
    gid_stream << gid;
    RCUTILS_LOG_DEBUG_NAMED(
      log_tag,
      "Adding topic '%s' with type '%s' for node ns='%s' name='%s' of participant '%s'",
      topic_name.c_str(),
      type_name.c_str(),
      namespace_.c_str(),
      node_name.c_str(),
      gid_stream.str().c_str());
  }
  topic_to_types_[topic_name].push_back(type_name);
  participant_to_nodes_to_topics_[gid][pair][topic_name].push_back(type_name);
  return true;
}

bool
TopicCache::remove_topic(
  const rmw_gid_t & gid,
  const std::string & namespace_,
  const std::string & node_name,
  const std::string & topic_name,
  const std::string & type_name)
{
  std::lock_guard<std::mutex> guard(mutex_);
  auto pair = std::make_pair<const std::string &, const std::string &>(namespace_, node_name);
  if (topic_to_types_.find(topic_name) == topic_to_types_.end()) {
    RCUTILS_LOG_DEBUG_NAMED(
      log_tag,
      "unexpected removal on topic '%s' with type '%s'",
      topic_name.c_str(), type_name.c_str());
    return false;
  }
  {
    auto & type_vec = topic_to_types_[topic_name];
    type_vec.erase(std::find(type_vec.begin(), type_vec.end(), type_name));
    if (type_vec.empty()) {
      topic_to_types_.erase(topic_name);
    }
  }

  const auto & gid_nodes_pair = participant_to_nodes_to_topics_.find(gid);
  if (gid_nodes_pair != participant_to_nodes_to_topics_.end()) {
    const auto & node_topics_pair = gid_nodes_pair->second.find(pair);
    if (node_topics_pair != gid_nodes_pair->second.end()) {
      const auto & topic_types_pair = node_topics_pair->second.find(topic_name);
      if (topic_types_pair != node_topics_pair->second.end()) {
        topic_types_pair->second.erase(std::find(
            topic_types_pair->second.begin(),
            topic_types_pair->second.end(),
            type_name));
        if (topic_types_pair->second.empty()) {
          node_topics_pair->second.erase(topic_name);
        }
        if (node_topics_pair->second.empty()) {
          gid_nodes_pair->second.erase(pair);
        }
        if (gid_nodes_pair->second.empty()) {
          participant_to_nodes_to_topics_.erase(gid);
        }
      }
    }
  } else {
    RCUTILS_LOG_DEBUG_NAMED(
      log_tag,
      "Unable to remove topic, does not exist '%s' with type '%s'",
      topic_name.c_str(), type_name.c_str());
  }
  return true;
}

rmw_ret_t
TopicCache::get_count(std::string topic_name, std::string (* mangle_topic)(std::string), size_t * count)
{
  std::lock_guard<std::mutex> guard(mutex_);
  assert(nullptr != mangle_topic);
  assert(nullptr != count);

  std::string fqdn = mangle_topic(topic_name);
  if ("" == fqdn) {
    return RMW_RET_ERROR;
  }
  const auto & it = topic_to_types_.find(fqdn);
  if (it != topic_to_types_.end()) {
    *count = it->second.size();
  } else {
    *count = 0;
  }
  return RMW_RET_OK;
}

using NamesAndTypes =
  std::vector<std::pair<std::string, std::reference_wrapper<const std::vector<std::string>>>>;

static
NamesAndTypes
__get_names_and_types(
  const TopicCache::TopicToTypes & topic_to_types,
  std::string (* demangle_topic)(const std::string &))
{
  NamesAndTypes topics;

  for (const auto & item : topic_to_types) {
    std::string demangled_topic_name = demangle_topic(item.first);
    if ("" != demangled_topic_name) {
      topics.emplace_back(std::move(demangled_topic_name), item.second);
    }
  }
  return topics;
}

static
NamesAndTypes
__get_names_and_types_by_node(
  const TopicCache::ParticipantNodeMap & participant_to_node_map,
  const rmw_gid_t & gid,
  const std::string & node_name,
  const std::string & namespace_,
  std::string (* demangle_topic)(const std::string &))
{
  NamesAndTypes topics;

  const auto & nodes_to_topics = participant_to_node_map.find(gid);
  if (nodes_to_topics == participant_to_node_map.end()) {
    return topics;
  }
  const auto & topic_to_types = nodes_to_topics->second.find(
    std::make_pair<const std::string &, const std::string &>(namespace_, node_name));
  if (topic_to_types == nodes_to_topics->second.end()) {
    return topics;
  }
  __get_names_and_types(topic_to_types->second, demangle_topic);
  return topics;
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
        item.second.get().size(),
        allocator);
      if (rcutils_ret != RCUTILS_RET_OK) {
        RMW_SET_ERROR_MSG(rcutils_get_error_string().str);
        rmw_ret = rmw_convert_rcutils_ret_to_rmw_ret(rcutils_ret);
        goto cleanup;
      }
    }
    size_t type_index = 0;
    for (const auto & type : item.second.get()) {
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
TopicCache::get_names_and_types_by_node(
  const rmw_gid_t & gid,
  const std::string & node_name,
  const std::string & namespace_,
  std::string (* demangle_topic)(const std::string &),
  std::string (* demangle_type)(const std::string &),
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * topic_names_and_types) const
{
  std::lock_guard<std::mutex> guard(mutex_);

  assert(demangle_topic);
  assert(demangle_type);
  assert(allocator);
  assert(topic_names_and_types);

  NamesAndTypes topics = __get_names_and_types_by_node(
    this->participant_to_nodes_to_topics_,
    gid,
    node_name,
    namespace_,
    demangle_topic);

  return __copy_data_to_results(
    topics,
    demangle_type,
    allocator,
    topic_names_and_types);
}

rmw_ret_t
TopicCache::get_names_and_types(
  std::string (* demangle_topic)(const std::string &),
  std::string (* demangle_type)(const std::string &),
  rcutils_allocator_t * allocator,
  rmw_names_and_types_t * topic_names_and_types) const
{
  std::lock_guard<std::mutex> guard(mutex_);

  assert(demangle_topic);
  assert(demangle_type);
  assert(allocator);
  assert(topic_names_and_types);

  NamesAndTypes topics = __get_names_and_types(
    this->topic_to_types_,
    demangle_topic);

  return __copy_data_to_results(
    topics,
    demangle_type,
    allocator,
    topic_names_and_types);
}

void
TopicCache::initialize_topic(const std::string & topic_name, TopicToTypes & topic_to_types)
{
  if (topic_to_types.find(topic_name) == topic_to_types.end()) {
    topic_to_types[topic_name] = std::vector<std::string>();
  }
}

void
TopicCache::initialize_node_topic_map(const NamespaceNamePair & pair, NodeTopicMap & map)
{
  if (map.find(pair) == map.end()) {
    map[pair] = TopicToTypes();
  }
}

void
TopicCache::initialize_participant_node_map(const rmw_gid_t & gid, ParticipantNodeMap & map)
{
  if (map.find(gid) == map.end()) {
    map[gid] = NodeTopicMap();
  }
}

std::ostream &
rmw_dds_common::operator<<(std::ostream & ostream, const TopicCache & topic_cache)
{
  std::lock_guard<std::mutex> guard(topic_cache.mutex_);

  std::ostringstream ss;
  ss << "Participant Info: " << std::endl;
  for (const auto & gid_node_map_pair : topic_cache.get_participant_to_nodes_to_topics()) {
    ss << "  gid: " << gid_node_map_pair.first << std::endl;
    for (const auto & node_topic_map_pair : gid_node_map_pair.second) {
      ss << "    Node:" << std::endl;
      ss << "      ns='" << node_topic_map_pair.first.first << "'" << std::endl;
      ss << "      name='" << node_topic_map_pair.first.second << "'" << std::endl;
      ss << "    Node:" << std::endl;
      ss << "      Topics: " << std::endl;
      for (const auto & topic_types_pair : node_topic_map_pair.second) {
        ss << "        " << topic_types_pair.first << ": ";
        std::copy(topic_types_pair.second.begin(), topic_types_pair.second.end(),
          std::ostream_iterator<std::string>(ostream, ","));
        ss << std::endl;
      }
    }
  }
  ss << "Cumulative TopicToTypes: " << std::endl;
  for (auto & elem : topic_cache.get_topic_to_types()) {
    ss << "  " << elem.first << ": ";
    std::copy(elem.second.begin(), elem.second.end(), std::ostream_iterator<std::string>(ostream,
      ","));
    ss << std::endl;
  }
  return ss << ss.str();
}
