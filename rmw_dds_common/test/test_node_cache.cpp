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

#include <set>
#include <string>
#include <tuple>
#include <vector>

#include "rmw/ret_types.h"

typedef std::string GUID_t;
#include "rmw_dds_common/node_cache.hpp"

using rmw_dds_common::NodeCache;

TEST(TestNodeCache, constructor_destructor)
{
  NodeCache node_cache;
}

using TestData =
  std::vector<std::tuple<GUID_t, std::vector<std::string>, std::vector<std::string>>>;

void check_names(
  TestData test_data,
  rcutils_string_array_t node_names,
  rcutils_string_array_t node_namespaces)
{
  std::set<size_t> checked_elements;
  for (const auto & elem : test_data) {
    const auto & original_names = std::get<1>(elem);
    const auto & original_namespaces = std::get<2>(elem);
    for (size_t i = 0; i < original_names.size(); i++) {
      for (size_t j = 0; j < node_names.size; j++) {
        if (original_names[i] == node_names.data[j] &&
          original_namespaces[i] == node_namespaces.data[j])
        {
          EXPECT_EQ(checked_elements.end(), checked_elements.find(j));
          checked_elements.insert(j);
          break;
        }
      }
    }
  }
}

TEST(TestNodeCache, common_usage)
{
  std::vector<std::tuple<GUID_t, std::vector<std::string>, std::vector<std::string>>> test_data = {
    {"guid1", {"ns1", "ns1", "ns2"}, {"node1", "node2", "node1"}},
    {"guid2", {"ns1", "ns2", "ns3", "ns4"}, {"node3", "node2", "node1", "node1"}},
  };
  NodeCache node_cache;
  for (const auto & elem : test_data) {
    node_cache.update_node_names(std::get<0>(elem), std::get<1>(elem), std::get<2>(elem));
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

  node_cache.delete_node_names("guid1");
  EXPECT_EQ(RMW_RET_OK, node_cache.get_number_of_nodes(nodes_number));
  EXPECT_EQ(4u, nodes_number);
  node_names = rcutils_get_zero_initialized_string_array();
  node_namespaces = rcutils_get_zero_initialized_string_array();
  node_cache.get_node_names(&node_names, &node_namespaces, &allocator);
  ASSERT_EQ(node_names.size, node_namespaces.size);
  std::vector<std::tuple<GUID_t, std::vector<std::string>, std::vector<std::string>>>
  current_data;
  current_data.emplace_back(test_data[1]);
  check_names(current_data, node_names, node_namespaces);
  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&node_names));
  EXPECT_EQ(RCUTILS_RET_OK, rcutils_string_array_fini(&node_namespaces));
}
