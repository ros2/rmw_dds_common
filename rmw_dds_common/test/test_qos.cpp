// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include "osrf_testing_tools_cpp/scope_exit.hpp"
#include "rmw/types.h"

#include "rmw_dds_common/qos.hpp"

static rmw_qos_profile_t
get_qos_profile_fixture()
{
  return {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };
}

TEST(test_qos, test_qos_profile_check_compatible_reliability)
{
  // Reliable pub, reliable sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }

  // Reliable pub, best effort sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }

  // Best effort pub, best effort sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }

  // Best effort pub, reliable sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_durability)
{
  // Volatile pub, volatile sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Volatile pub, transient local sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Transient local pub, transient local sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Transient local pub, volatile sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_deadline)
{
  // No deadline pub, no deadline sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.deadline = RMW_QOS_DEADLINE_DEFAULT;
    sub_qos.deadline = RMW_QOS_DEADLINE_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }

  // No deadline pub, deadline sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.deadline = RMW_QOS_DEADLINE_DEFAULT;
    sub_qos.deadline = {1, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }

  // Deadline pub > deadline sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.deadline = {1, 1};
    sub_qos.deadline = {1, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Deadline pub == deadline sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.deadline = {1, 1};
    sub_qos.deadline = {1, 1};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Deadline pub < deadline sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.deadline = {1, 1};
    sub_qos.deadline = {2, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_liveliness)
{
  // Automatic pub, automatic sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Automatic pub, manual sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Manual pub, manual sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Manual pub, automatic sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_liveliness_lease_duration)
{
  // No duration pub, no duration sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    sub_qos.liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // No duration pub, some duration sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    sub_qos.liveliness_lease_duration = {1, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Lease duration pub == lease duration sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness_lease_duration = {1, 0};
    sub_qos.liveliness_lease_duration = {1, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Lease duration pub > lease duration sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness_lease_duration = {1, 1};
    sub_qos.liveliness_lease_duration = {1, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Lease duration pub < lease duration sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness_lease_duration = {1, 1};
    sub_qos.liveliness_lease_duration = {2, 1};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_system_default)
{
  // Some policies set to "system default" should cause a warning

  // Pub best effort, sub system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub system default, sub reliable
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both reliability system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub volatile, sub system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub system default, sub transient local
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both durability system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub automatic, sub system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub system default, sub manual by topic
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both liveliness system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_unknown)
{
  // Some policies set to "unknown" should cause a warning

  // Pub best effort, sub unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub unknown, sub reliable
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both reliability unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub volatile, sub unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub unknown, sub transient local
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both durability unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub automatic, sub unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub unknown, sub manual by topic
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both liveliness unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_no_reason)
{
  // Compatible
  {
    rmw_qos_compatibility_type_t compatible;
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, nullptr, 0u);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
  }
  // Incompatible
  {
    rmw_qos_compatibility_type_t compatible;
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, nullptr, 0u);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
  }
}

TEST(test_qos, test_qos_profile_check_compatible_reason_buffer_too_small)
{
  // We expect a message larger than 10 characters
  char reason[10];
  size_t reason_size = 10u;
  rmw_qos_compatibility_type_t compatible;
  rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
  rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
  pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
    pub_qos, sub_qos, &compatible, reason, reason_size);
  EXPECT_EQ(ret, RMW_RET_OK);
  EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
  // Expect the first 10 characters including the terminating null character
  EXPECT_STREQ(reason, "ERROR: Be");
}

TEST(test_qos, test_qos_profile_check_compatible_reason_buffer_size_zero)
{
  char reason[10] = "untouched";
  // With reason size zero, we don't expect any message to be written
  size_t reason_size = 0u;
  rmw_qos_compatibility_type_t compatible;
  rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
  rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
  pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
    pub_qos, sub_qos, &compatible, reason, reason_size);
  EXPECT_EQ(ret, RMW_RET_OK);
  EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
  EXPECT_STREQ(reason, "untouched");
}

TEST(test_qos, test_qos_profile_check_compatible_invalid)
{
  // Null compatible parameter
  {
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, nullptr, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
  }
  // Null reason, but non-zero reason_size
  {
    rmw_qos_compatibility_type_t compatible;
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, nullptr, 3u);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
  }
}

TEST(test_qos, test_qos_profile_get_most_compatible_for_subscription)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  // One publisher profile
  {
    rmw_topic_endpoint_info_array_t publishers_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_ret_t init_ret = rmw_topic_endpoint_info_array_init_with_size(
      &publishers_info, 1u, &allocator);
    ASSERT_EQ(init_ret, RMW_RET_OK);
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    {
      EXPECT_EQ(rmw_topic_endpoint_info_array_fini(&publishers_info, &allocator), RMW_RET_OK);
    });

    publishers_info.info_array[0].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };
    rmw_qos_profile_t subscription_profile = get_qos_profile_fixture();
    subscription_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    subscription_profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_most_compatible_for_subscription(
      &publishers_info, &subscription_profile, nullptr, nullptr);

    EXPECT_EQ(ret, RMW_RET_OK);
    // Expect the reliability and durability to change, matching publisher QoS
    const rmw_qos_profile_t & publisher_profile = publishers_info.info_array[0].qos_profile;
    EXPECT_EQ(subscription_profile.reliability, publisher_profile.reliability);
    EXPECT_EQ(subscription_profile.durability, publisher_profile.durability);
    EXPECT_TRUE(rmw_time_equal(subscription_profile.deadline, publisher_profile.deadline));
  }
  // More than one publisher profile
  {
    rmw_topic_endpoint_info_array_t publishers_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_ret_t init_ret = rmw_topic_endpoint_info_array_init_with_size(
      &publishers_info, 3u, &allocator);
    ASSERT_EQ(init_ret, RMW_RET_OK);
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    {
      EXPECT_EQ(rmw_topic_endpoint_info_array_fini(&publishers_info, &allocator), RMW_RET_OK);
    });

    publishers_info.info_array[0].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };
    publishers_info.info_array[1].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,  // should result in "best effort" for subscription
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      {3u, 0u},  // this deadline should appear in subscription QoS because it is largest
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };
    publishers_info.info_array[2].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE,  // should result in "volatile" for subscription
      {2u, 0u},
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };
    rmw_qos_profile_t subscription_profile = get_qos_profile_fixture();
    rmw_topic_endpoint_info_array_t incompatible_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_most_compatible_for_subscription(
      &publishers_info, &subscription_profile, &incompatible_info, &allocator);

    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(subscription_profile.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    EXPECT_EQ(subscription_profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
    EXPECT_TRUE(rmw_time_equal(subscription_profile.deadline, {3u, 0u}));
    EXPECT_EQ(incompatible_info.size, 0u);
  }
}

TEST(test_qos, test_qos_profile_get_most_compatible_for_subscription_invalid_arguments)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rmw_topic_endpoint_info_array_t publishers_info =
    rmw_get_zero_initialized_topic_endpoint_info_array();
  rmw_ret_t init_ret = rmw_topic_endpoint_info_array_init_with_size(
    &publishers_info, 1u, &allocator);
  ASSERT_EQ(init_ret, RMW_RET_OK);
  rmw_qos_profile_t subscription_profile = get_qos_profile_fixture();

  // NULL publishers_info
  {
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_most_compatible_for_subscription(
      nullptr, &subscription_profile, nullptr, nullptr);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
  }
  // Zero length
  {
    rmw_topic_endpoint_info_array_t zeroed_publishers_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_most_compatible_for_subscription(
      &zeroed_publishers_info, &subscription_profile, nullptr, nullptr);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
  }
  // NULL subscription profile
  {
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_most_compatible_for_subscription(
      &publishers_info, nullptr, nullptr, nullptr);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
  }
  // Valid incompatible info and invalid allocator
  {
    rmw_topic_endpoint_info_array_t incompatible_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_most_compatible_for_subscription(
      &publishers_info, &subscription_profile, &incompatible_info, nullptr);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
  }
  // Non-zeroed incompatible info
  {
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_most_compatible_for_subscription(
      &publishers_info, &subscription_profile, &publishers_info, &allocator);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
  }
}
