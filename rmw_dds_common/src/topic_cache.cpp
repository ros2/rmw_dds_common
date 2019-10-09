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
#include <iterator>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "rmw_dds_common/gid_utils.hpp"
#include "rmw_dds_common/topic_cache.hpp"

using rmw_dds_common::TopicCache;
using rmw_dds_common::operator<<;

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
  auto pair = std::make_pair(
    const_cast<std::string &>(namespace_),
    const_cast<std::string &>(node_name));
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
      "rmw_fastrtps_shared_cpp",
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
  auto pair = std::make_pair(
    const_cast<std::string &>(namespace_),
    const_cast<std::string &>(node_name));
  if (topic_to_types_.find(topic_name) == topic_to_types_.end()) {
    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_fastrtps_shared_cpp",
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
      "rmw_fastrtps_shared_cpp",
      "Unable to remove topic, does not exist '%s' with type '%s'",
      topic_name.c_str(), type_name.c_str());
  }
  return true;
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
