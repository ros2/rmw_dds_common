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

#include "rmw_dds_common/qos.hpp"

#include <cstring>

#include "rcutils/snprintf.h"
#include "rmw/error_handling.h"
#include "rmw/qos_profiles.h"

namespace rmw_dds_common
{

static bool
operator==(rmw_time_t t1, rmw_time_t t2)
{
  return t1.sec == t2.sec && t1.nsec == t2.nsec;
}

static bool
operator!=(rmw_time_t t1, rmw_time_t t2)
{
  return !(t1 == t2);
}

static bool
operator<(rmw_time_t t1, rmw_time_t t2)
{
  if (t1.sec < t2.sec) {
    return true;
  } else if (t1.sec == t2.sec && t1.nsec < t2.nsec) {
    return true;
  }
  return false;
}

// Returns RMW_RET_OK if successful or no buffer was provided
// Returns RMW_RET_ERROR if there as an error copying the message to the buffer
static rmw_ret_t
_append_to_buffer(const char * message, char * buffer, size_t buffer_size)
{
  // Only write if a buffer is provided
  if (!buffer || buffer_size == 0u) {
    return RMW_RET_OK;
  }
  // Determine available space left in buffer
  size_t offset = strnlen(buffer, buffer_size);
  size_t write_size = buffer_size - offset;
  int snprintf_ret = rcutils_snprintf(buffer + offset, write_size, "%s", message);
  if (snprintf_ret < 0) {
    RMW_SET_ERROR_MSG("failed to append to character buffer");
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

rmw_ret_t
qos_profile_check_compatible(
  const rmw_qos_profile_t publisher_qos,
  const rmw_qos_profile_t subscription_qos,
  rmw_qos_compatibility_type_t * compatibility,
  char * reason,
  size_t reason_size)
{
  if (!compatibility) {
    RMW_SET_ERROR_MSG("compatibility parameter is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (!reason && reason_size != 0u) {
    RMW_SET_ERROR_MSG("reason parameter is null, but reason_size parameter is not zero");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Presume profiles are compatible until proven otherwise
  *compatibility = RMW_QOS_COMPATIBILITY_OK;

  // If there are any "unknown" values, then there is an error
  if (RMW_QOS_POLICY_RELIABILITY_UNKNOWN == publisher_qos.reliability) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    RMW_SET_ERROR_MSG("Publisher reliability is unknown");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (RMW_QOS_POLICY_RELIABILITY_UNKNOWN == subscription_qos.reliability) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    RMW_SET_ERROR_MSG("Subscription reliability is unknown");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (RMW_QOS_POLICY_DURABILITY_UNKNOWN == publisher_qos.durability) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    RMW_SET_ERROR_MSG("Publisher durability is unknown");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (RMW_QOS_POLICY_DURABILITY_UNKNOWN == subscription_qos.durability) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    RMW_SET_ERROR_MSG("Subscription durability is unknown");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (RMW_QOS_POLICY_LIVELINESS_UNKNOWN == publisher_qos.liveliness) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    RMW_SET_ERROR_MSG("Publisher liveliness is unknown");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (RMW_QOS_POLICY_LIVELINESS_UNKNOWN == subscription_qos.liveliness) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    RMW_SET_ERROR_MSG("Subscription liveliness is unknown");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Best effort publisher and reliable subscription
  if (publisher_qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
    subscription_qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE)
  {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t append_ret = _append_to_buffer(
      "ERROR: Best effort publisher and reliable subscription;",
      reason,
      reason_size);
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  // Volatile publisher and transient local subscription
  if (publisher_qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&
    subscription_qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
  {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t append_ret = _append_to_buffer(
      "ERROR: Volatile publisher and transient local subscription;",
      reason,
      reason_size);
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  const rmw_time_t & pub_deadline = publisher_qos.deadline;
  const rmw_time_t & sub_deadline = subscription_qos.deadline;
  const rmw_time_t deadline_default = RMW_QOS_DEADLINE_DEFAULT;

  // No deadline for publisher and deadline for subscription
  if (pub_deadline == deadline_default && sub_deadline != deadline_default) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t ret = _append_to_buffer(
      "ERROR: Subscription has a deadline, but publisher does not;",
      reason,
      reason_size);
    if (RMW_RET_OK != ret) {
      return ret;
    }
  }

  // Subscription deadline is less than publisher deadline
  if (pub_deadline != deadline_default && sub_deadline != deadline_default) {
    if (sub_deadline < pub_deadline) {
      *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
      rmw_ret_t append_ret = _append_to_buffer(
        "ERROR: Subscription deadline is less than publisher deadline;",
        reason,
        reason_size);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    }
  }

  // Automatic liveliness for publisher and manual by topic for subscription
  if (publisher_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_AUTOMATIC &&
    subscription_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
  {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t append_ret = _append_to_buffer(
      "ERROR: Publisher's liveliness is automatic and subscription's is manual by topic;",
      reason,
      reason_size);
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  const rmw_time_t & pub_lease = publisher_qos.liveliness_lease_duration;
  const rmw_time_t & sub_lease = subscription_qos.liveliness_lease_duration;
  const rmw_time_t lease_default = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;

  // No lease duration for publisher and lease duration for subscription
  if (pub_lease == lease_default && sub_lease != lease_default) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t append_ret = _append_to_buffer(
      "ERROR: Subscription has a liveliness lease duration, but publisher does not;",
      reason,
      reason_size);
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  // Subscription lease duration is less than publisher lease duration
  if (pub_lease != lease_default && sub_lease != lease_default) {
    if (sub_lease < pub_lease) {
      *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
      rmw_ret_t append_ret = _append_to_buffer(
        "ERROR: Subscription liveliness lease duration is less than publisher;",
        reason,
        reason_size);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    }
  }

  // Only check for warnings if there are no errors
  if (RMW_QOS_COMPATIBILITY_OK == *compatibility) {
    // Reliability for publisher is "system default" and subscription is reliable
    if (publisher_qos.reliability == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT &&
      subscription_qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    {
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t append_ret = _append_to_buffer(
        "WARNING: Reliable subscription, but publisher is system default;",
        reason,
        reason_size);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    } else {
      // Reliability for publisher is best effort and subscription is "system default"
      if (publisher_qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
        subscription_qos.reliability == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT)
      {
        *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
        rmw_ret_t append_ret = _append_to_buffer(
          "WARNING: Best effort publisher, but subscription is system default;",
          reason,
          reason_size);
        if (RMW_RET_OK != append_ret) {
          return append_ret;
        }
      }
    }

    // Durability for publisher is "system default" and subscription is transient local
    if (publisher_qos.durability == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT &&
      subscription_qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    {
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t ret = _append_to_buffer(
        "WARNING: Transient local subscription, but publisher is system default;",
        reason,
        reason_size);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    } else {
      // Durability for publisher is volatile and subscription is "system default"
      if (publisher_qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&
        subscription_qos.durability == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT)
      {
        *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
        rmw_ret_t ret = _append_to_buffer(
          "WARNING: Volatile publisher, but subscription is system default;",
          reason,
          reason_size);
        if (RMW_RET_OK != ret) {
          return ret;
        }
      }
    }

    // Automatic liveliness for publisher and "system default" for subscription
    if (publisher_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_AUTOMATIC &&
      subscription_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT)
    {
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t ret = _append_to_buffer(
        "WARNING: Publisher's liveliness is automatic, but subscription's is system default;",
        reason,
        reason_size);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    } else {
      if (publisher_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT &&
        subscription_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
      {
        *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
        rmw_ret_t ret = _append_to_buffer(
          "WARNING: Subscription's liveliness is manual by topic, but publisher's is system "
          "default;",
          reason,
          reason_size);
        if (RMW_RET_OK != ret) {
          return ret;
        }
      }
    }
  }

  return RMW_RET_OK;
}

}  // namespace rmw_dds_common
