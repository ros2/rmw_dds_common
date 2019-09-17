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

#ifndef RMW_DDS_COMMON__RMW_CONTEXT_IMPL_H_
#define RMW_DDS_COMMON__RMW_CONTEXT_IMPL_H_

#include "rmw/init.h"
#include "rmw/types.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct rmw_context_impl_t
{
  const rmw_gid_t * gid;
  rmw_publisher_t * pub;
  void * topic_cache;
  void * node_cache;
  void * data;
} rmw_context_impl_t;


/// Init context implementation
/**
 * Init the context implementation in the passed context.
 *
 * \param[in] gid Gid of the DDS participant, or similar unique identifier.
 *    Should be a valid pointer previously allocated.
 * \param[in] state_publisher Already initialized publisher, that will be later used to
 *   publish ParticipantCustomInfo messages.
 * \param[in] data Implementation specific detail.
 *   For DDS based implementation it may be only a Participant.
 * \param[inout] context Context where the implementation is inited.
 *   For DDS based implementation it may be only a Participant.
 * \return RMW_RET_BAD_ALLOC, or
 * \return RMW_RET_OK.
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t
rmw_dds_common_context_impl_init(
  const rmw_gid_t * gid,
  rmw_publisher_t * state_publisher,
  void * data,
  rmw_context_t * context);

/// Finish context implementation
/**
 * Finish the context implementation in the passed context.
 * If gid or data were allocated dinamically before calling
 * `rmw_init_context_impl`, they should be freed before calling
 * this function.
 *
 * \return RMW_RET_BAD_ALLOC, or
 * \return RMW_RET_OK.
 */
RMW_DDS_COMMON_PUBLIC
void
rmw_dds_common_context_impl_fini(rmw_context_t * context);

#ifdef __cplusplus
}
#endif

#endif  // RMW_DDS_COMMON__RMW_CONTEXT_IMPL_H_
