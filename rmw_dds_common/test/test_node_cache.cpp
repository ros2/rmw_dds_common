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
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "rmw/types.h"
#include "rmw/ret_types.h"

#include "rmw_dds_common/gid_utils.hpp"
#include "rmw_dds_common/node_cache.hpp"

using rmw_dds_common::NodeCache;

TEST(TestNodeCache, constructor_destructor)
{
  NodeCache node_cache;
}

using TestData =
  std::vector<std::pair<rmw_gid_t, NodeCache::NodeInfoVector>>;

void check_names(
  TestData test_data,
  rcutils_string_array_t node_names,
  rcutils_string_array_t node_namespaces)
{
  std::set<size_t> checked_elements;
  for (const auto & elem : test_data) {
    const auto & original_names = std::get<1>(elem);
    for (const auto & node_info : original_names) {
      for (size_t j = 0; j < node_names.size; j++) {
        if (node_info.node_name == node_names.data[j] &&
          node_info.node_namespace == node_namespaces.data[j])
        {
          EXPECT_EQ(checked_elements.end(), checked_elements.find(j));
          checked_elements.insert(j);
          break;
        }
      }
    }
  }
}

rmw_dds_common::msg::NodeCustomInfo
new_node_info(std::string ns, std::string name)
{
  rmw_dds_common::msg::NodeCustomInfo info;
  info.node_namespace = ns;
  info.node_name = name;
  return info;
}

rmw_gid_t
generate_gid(std::string data)
{
  rmw_gid_t gid;
  std::strncpy(reinterpret_cast<char *>(gid.data), data.c_str(), RMW_GID_STORAGE_SIZE);
  return gid;
}

template<class T0, class ... Ts>
auto
make_vector(T0 && first, Ts && ... args)
{
  using first_type = std::decay_t<T0>;
  return std::vector<first_type>{
    std::forward<T0>(first),
    std::forward<Ts>(args)...
  };
}

TestData
generate_test_data()
{
  TestData test_data;
  rmw_gid_t gid1 = generate_gid("gid1");
  rmw_gid_t gid2 = generate_gid("gid2");
  test_data.emplace_back(
    gid1,
    make_vector(
      new_node_info("ns1", "node1"),
      new_node_info("ns1", "node2"),
      new_node_info("ns2", "node1")));
  test_data.emplace_back(
    gid2,
    make_vector(
      new_node_info("ns1", "node3"),
      new_node_info("ns2", "node2"),
      new_node_info("ns3", "node1"),
      new_node_info("ns4", "node1")));
  return test_data;
}

TEST(TestNodeCache, common_usage)
{
  TestData test_data = generate_test_data();
  NodeCache node_cache;
  for (const auto & elem : test_data) {
    node_cache.update_node_names(std::get<0>(elem), std::get<1>(elem));
  }

  size_t nodes_number;
  EXPECT_EQ(RMW_RET_OK, node_cache.get_number_of_nodes(nodes_number));
  EXPECT_EQ(7u, nodes_number);
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rcutils_string_array_t node_names = rcutils_get_zero_initialized_string_array();
  rcutils_string_array_t node_namespaces = rcutils_get_zero_initialized_string_array();
  node_cache.get_node_names(&node_names, &node_namespaces, &allocator);
  ASSERT_EQ(node_names.size, node_namespaces.size);
  check_names(test_data, node_names, node_namespaces);
  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&node_names));
  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&node_namespaces));

  node_cache.delete_node_names(test_data[0].first);
  EXPECT_EQ(RMW_RET_OK, node_cache.get_number_of_nodes(nodes_number));
  EXPECT_EQ(4u, nodes_number);
  node_names = rcutils_get_zero_initialized_string_array();
  node_namespaces = rcutils_get_zero_initialized_string_array();
  node_cache.get_node_names(&node_names, &node_namespaces, &allocator);
  ASSERT_EQ(node_names.size, node_namespaces.size);
  TestData current_data;
  current_data.emplace_back(test_data[1]);
  check_names(current_data, node_names, node_namespaces);
  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&node_names));
  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&node_namespaces));
}
