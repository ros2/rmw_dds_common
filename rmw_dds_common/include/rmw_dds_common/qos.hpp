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

#ifndef RMW_DDS_COMMON__QOS_HPP_
#define RMW_DDS_COMMON__QOS_HPP_

#include "rmw/qos_profiles.h"
#include "rmw/types.h"

#include "rmw_dds_common/visibility_control.h"

namespace rmw_dds_common
{

/// Check if two QoS profiles are compatible
/**
 * Two QoS profiles are compatible if a publisher and subcription
 * using the QoS policies can communicate with each other.
 *
 * This implements the rmw API \ref rmw_qos_profile_check_compatible().
 * See \ref rmw_qos_profile_check_compatible() for more information.
 *
 * \param[in] publisher_qos: The QoS profile used for a publisher.
 * \param[in] subscription_qos: The QoS profile used for a subscription.
 * \param[out] compatibility: `RMW_QOS_COMPATIBILITY_OK` if the QoS profiles are compatible, or
 *   `RMW_QOS_COMPATIBILITY_WARNING` if the QoS profiles might be compatible, or
 *   `RMW_QOS_COMPATIBILITY_ERROR` if the QoS profiles are not compatible.
 * \param[out] reason: A detailed reason for a QoS incompatibility or potential incompatibility.
 *   Must be pre-allocated by the caller.
 *   This parameter is optional and may be set to `nullptr` if the reason information is not
 *   desired.
 * \param[in] reason_size: Size of the string buffer `reason`, if one is provided.
 *   If `reason` is `nullptr`, then this parameter must be zero.
 * \return `RMW_RET_OK` if the check was successful, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `compatiblity` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `reason` is `nullptr` and  `reason_size` is not zero, or
 * \return `RMW_RET_ERROR` if there is an unexpected error.
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t
qos_profile_check_compatible(
  const rmw_qos_profile_t publisher_qos,
  const rmw_qos_profile_t subscription_qos,
  rmw_qos_compatibility_type_t * compatibility,
  char * reason,
  size_t reason_size);

/// Get compatible QoS policies for a subscription.
/**
 * Given one or more publisher QoS profiles, return a QoS profile for a subscirption
 * that is compatible with the majority of the publisher profiles while maintaining the highest
 * level of service possible.
 *
 * This implements the rmw API \ref rmw_qos_profile_get_most_compatible_for_subscription().
 * See \ref rmw_qos_profile_get_most_compatible_for_subscription() for more information.
 *
 * \param[in] publisher_profiles: An array of QoS profiles used for publishers.
 * \param[out] subscription_profile: QoS policies that are compatible with the majorty of
 *   the input publisher profiles.
 * \param[out] compatible_publisher_profiles: An array of boolean values indicating if the
 *   corresponding QoS profile at the same index in the publisher profiles array is compatible
 *   with the resultant subscription QoS profile or not.
 *   This parameter is optional and may be `nullptr`.
 *   If provided, it must be the same length as the publisher profiles array.
 * \return `RMW_RET_OK` if the operation was successful, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `publisher_profiles` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `publisher_profiles_length` is 0, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `subscription_profile` is `nullptr`, or
 * \return `RMW_RET_ERROR` if there is an unexpected error.
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t
qos_profile_get_most_compatible_for_subscription(
  const rmw_qos_profile_t * publisher_profiles,
  size_t publisher_profiles_length,
  rmw_qos_profile_t * subscription_profile,
  bool * compatible_publisher_profiles);

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__QOS_HPP_
