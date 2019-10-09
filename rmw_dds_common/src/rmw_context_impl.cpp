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

#include "rmw_dds_common/locked_object.hpp"
#include "rmw_dds_common/node_cache.hpp"
#include "rmw_dds_common/topic_cache.hpp"
#include "rmw_dds_common/rmw_context_impl.h"

extern "C"
{
using rmw_dds_common::NodeCache;
using rmw_dds_common::TopicCache;
using rmw_dds_common::LockedObject;

rmw_ret_t
rmw_dds_common_context_impl_init(rmw_context_t * context)
{
  NodeCache * node_cache = nullptr;
  LockedObject<TopicCache> * reader_topic_cache = nullptr;
  LockedObject<TopicCache> * writer_topic_cache = nullptr;

  context->impl = static_cast<rmw_context_impl_t *>(rmw_allocate(sizeof(rmw_context_impl_t)));
  if (nullptr == context->impl) {
    goto fail;
  }
  node_cache = static_cast<NodeCache *>(rmw_allocate(sizeof(NodeCache)));
  if (nullptr == node_cache) {
    goto fail;
  }
  reader_topic_cache = static_cast<LockedObject<TopicCache> *>(
    rmw_allocate(sizeof(LockedObject<TopicCache>)));
  if (nullptr == reader_topic_cache) {
    goto fail;
  }
  writer_topic_cache = static_cast<LockedObject<TopicCache> *>(
    rmw_allocate(sizeof(LockedObject<TopicCache>)));
  if (nullptr == writer_topic_cache) {
    goto fail;
  }

  RMW_TRY_PLACEMENT_NEW(node_cache, node_cache, goto fail, NodeCache, );
  RMW_TRY_PLACEMENT_NEW(
    reader_topic_cache,
    reader_topic_cache,
    goto fail,
    LockedObject<TopicCache>,
  );
  RMW_TRY_PLACEMENT_NEW(
    writer_topic_cache,
    writer_topic_cache,
    goto fail,
    LockedObject<TopicCache>,
  );
  context->impl->node_cache = node_cache;
  context->impl->reader_topic_cache = reader_topic_cache;
  context->impl->writer_topic_cache = writer_topic_cache;

  return RMW_RET_OK;
fail:
  delete context->impl;
  delete node_cache;
  delete reader_topic_cache;
  delete writer_topic_cache;
  return RMW_RET_BAD_ALLOC;
}

void
rmw_dds_common_context_impl_fini(rmw_context_t * context)
{
  delete reinterpret_cast<NodeCache *>(context->impl->node_cache);
  delete reinterpret_cast<LockedObject<TopicCache> *>(context->impl->reader_topic_cache);
  delete reinterpret_cast<LockedObject<TopicCache> *>(context->impl->writer_topic_cache);
  context->impl->node_cache = nullptr;
  context->impl->reader_topic_cache = nullptr;
  context->impl->writer_topic_cache = nullptr;
}
}  // extern "C"
