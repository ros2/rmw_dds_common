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

#include "rmw/allocators.h"
#include "rmw/init.h"
#include "rmw/types.h"
#include "rmw/impl/cpp/macros.hpp"

#include "rmw_dds_common/node_cache.hpp"
#include "rmw_dds_common/topic_cache.hpp"
#include "rmw_dds_common/rmw_context_impl.h"

extern "C"
{
using rmw_dds_common::NodeCache;
using rmw_dds_common::TopicCache;

rmw_ret_t
rmw_dds_common_context_impl_init(
  const rmw_gid_t * gid,
  rmw_publisher_t * state_publisher,
  void * data,
  rmw_context_t * context)
{
  NodeCache * node_cache = nullptr;
  TopicCache * topic_cache = nullptr;
  context->impl = reinterpret_cast<rmw_context_impl_t *>(rmw_allocate(sizeof(rmw_context_impl_t)));
  if (nullptr == context->impl) {
    goto fail;
  }
  RMW_TRY_PLACEMENT_NEW(node_cache, node_cache, goto fail, NodeCache, );
  RMW_TRY_PLACEMENT_NEW(topic_cache, topic_cache, goto fail, TopicCache, );
  context->impl->gid = gid;
  node_cache->add_gid(*gid);
  context->impl->node_cache = node_cache;
  context->impl->topic_cache = topic_cache;
  context->impl->data = data;
  context->impl->pub = state_publisher;

fail:
  delete context->impl;
  delete node_cache;
  delete topic_cache;
  return RMW_RET_BAD_ALLOC;
}

void
rmw_dds_common_context_impl_fini(rmw_context_t * context)
{
  delete reinterpret_cast<NodeCache *>(context->impl->node_cache);
  delete reinterpret_cast<TopicCache *>(context->impl->topic_cache);
  context->impl->gid = nullptr;
  context->impl->node_cache = nullptr;
  context->impl->topic_cache = nullptr;
  context->impl->data = nullptr;
  context->impl->pub = nullptr;
}
}  // extern "C"
