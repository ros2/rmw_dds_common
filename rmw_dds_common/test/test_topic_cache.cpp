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

#include <gtest/gtest.h>

#include <cstring>
#include <functional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rmw/types.h"

#include "rmw_dds_common/topic_cache.hpp"

using rmw_dds_common::TopicCache;
using rmw_dds_common::operator==;

TEST(TestTopicCache, construnctor_destructor) {
  TopicCache topic_cache;
}

void check_if_topic_in_map(
  std::string topic,
  std::string type,
  TopicCache::TopicToTypes topic_to_types)
{
  ASSERT_LE(1u, topic_to_types.size());
  const auto & topic_types_pair = topic_to_types.find(topic);
  ASSERT_NE(topic_to_types.end(), topic_types_pair);
  EXPECT_EQ(topic, topic_types_pair->first);
  ASSERT_LE(1u, topic_types_pair->second.size());
  EXPECT_NE(
    std::find(
      topic_types_pair->second.begin(),
      topic_types_pair->second.end(),
      type),
    topic_types_pair->second.end());
}

size_t get_number_of_topics(TopicCache::TopicToTypes topic_to_types)
{
  size_t ret = 0;
  for (const auto & element : topic_to_types) {
    ret += element.second.size();
  }
  return ret;
}

void check_if_topic_in_map(
  rmw_gid_t gid,
  std::string namespace_,
  std::string name,
  std::string topic,
  std::string type,
  TopicCache::ParticipantNodeMap participant_map)
{
  TopicCache::NamespaceNamePair name_pair = std::make_pair(namespace_, name);
  ASSERT_LE(1u, participant_map.size());
  const auto & gid_nodes_map_pair = participant_map.find(gid);
  ASSERT_NE(participant_map.end(), gid_nodes_map_pair);
  EXPECT_EQ(gid, gid_nodes_map_pair->first);
  ASSERT_LE(1u, gid_nodes_map_pair->second.size());
  const auto & node_topics_map_pair = gid_nodes_map_pair->second.find(name_pair);
  ASSERT_NE(gid_nodes_map_pair->second.end(), node_topics_map_pair);
  EXPECT_EQ(name_pair, node_topics_map_pair->first);
  ASSERT_LE(1u, node_topics_map_pair->second.size());
  const auto & topic_types_pair = node_topics_map_pair->second.find(topic);
  ASSERT_NE(node_topics_map_pair->second.end(), topic_types_pair);
  ASSERT_LE(1u, topic_types_pair->second.size());
  EXPECT_NE(
    std::find(
      topic_types_pair->second.begin(),
      topic_types_pair->second.end(),
      type),
    topic_types_pair->second.end());
}

size_t get_number_of_topics(TopicCache::ParticipantNodeMap participant_map)
{
  size_t ret = 0;
  for (const auto & node_map_pair : participant_map) {
    for (const auto & topic_map_pair : node_map_pair.second) {
      for (const auto & element : topic_map_pair.second) {
        ret += element.second.size();
      }
    }
  }
  return ret;
}

rmw_gid_t
generate_gid(std::string data)
{
  rmw_gid_t gid;
  std::strncpy(reinterpret_cast<char *>(gid.data), data.c_str(), RMW_GID_STORAGE_SIZE);
  return gid;
}

TEST(TestTopicCache, add_remove_one_topic) {
  TopicCache topic_cache;
  rmw_gid_t gid = generate_gid("my_fake_gid");
  topic_cache.add_topic(gid, "my_ns", "my_node", "my_topic", "my_type");
  check_if_topic_in_map("my_topic", "my_type", topic_cache.get_topic_to_types());
  EXPECT_EQ(1u, get_number_of_topics(topic_cache.get_topic_to_types()));
  check_if_topic_in_map(
    gid,
    "my_ns",
    "my_node",
    "my_topic",
    "my_type",
    topic_cache.get_participant_to_nodes_to_topics());
  EXPECT_EQ(1u, get_number_of_topics(topic_cache.get_participant_to_nodes_to_topics()));

  topic_cache.remove_topic(gid, "my_ns", "my_node", "my_topic", "my_type");
  EXPECT_EQ(0u, topic_cache.get_topic_to_types().size());
  EXPECT_EQ(0u, topic_cache.get_participant_to_nodes_to_topics().size());
}

using TopicInfo = std::tuple<rmw_gid_t, std::string, std::string, std::string, std::string>;
TEST(TestTopicCache, add_remove_multiple_topics) {
  TopicCache topic_cache;
  rmw_gid_t gid0 = generate_gid("gid0");
  rmw_gid_t gid1 = generate_gid("gid1");

  std::vector<TopicInfo> topic_info = {
    {gid0, "ns", "node0", "topic0", "type0"},
    {gid0, "ns", "node0", "topic0", "type0"},
    {gid0, "ns", "node1", "topic0", "type0"},
    {gid0, "ns", "node1", "topic1", "type0"},
    {gid0, "ns", "node1", "topic1", "type1"},
    {gid1, "ns", "node2", "topic2", "type2"},
    {gid1, "ns", "node2", "topic3", "type3"},
  };
  for (const auto & elem : topic_info) {
    topic_cache.add_topic(
      std::get<0>(elem),
      std::get<1>(elem),
      std::get<2>(elem),
      std::get<3>(elem),
      std::get<4>(elem));
  }
  for (const auto & elem : topic_info) {
    check_if_topic_in_map(
      std::get<3>(elem),
      std::get<4>(elem),
      topic_cache.get_topic_to_types());
    check_if_topic_in_map(
      std::get<0>(elem),
      std::get<1>(elem),
      std::get<2>(elem),
      std::get<3>(elem),
      std::get<4>(elem),
      topic_cache.get_participant_to_nodes_to_topics());
  }
  ASSERT_EQ(7u, get_number_of_topics(topic_cache.get_topic_to_types()));
  ASSERT_EQ(7u, get_number_of_topics(topic_cache.get_participant_to_nodes_to_topics()));

  for (const auto & elem : topic_info) {
    topic_cache.remove_topic(
      std::get<0>(elem),
      std::get<1>(elem),
      std::get<2>(elem),
      std::get<3>(elem),
      std::get<4>(elem));
  }
  EXPECT_EQ(0u, topic_cache.get_topic_to_types().size());
  EXPECT_EQ(0u, topic_cache.get_participant_to_nodes_to_topics().size());
}
