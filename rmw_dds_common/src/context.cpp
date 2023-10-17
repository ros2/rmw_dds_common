// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include "rmw_dds_common/context.hpp"

namespace rmw_dds_common
{

rmw_ret_t Context::update_node_graph(
  const std::string & name, const std::string & namespace_,
  publish_callback_t publish_callback)
{
  std::lock_guard<std::mutex> guard(node_update_mutex_new);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.add_node(gid, name, namespace_);

  return publish_callback(pub, static_cast<void *>(&msg));
}

rmw_ret_t Context::destroy_node_graph(
  const std::string & name, const std::string & namespace_,
  publish_callback_t publish_callback)
{
  std::lock_guard<std::mutex> guard(node_update_mutex_new);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.remove_node(gid, name, namespace_);

  return publish_callback(pub, static_cast<void *>(&msg));
}

rmw_ret_t Context::update_subscriber_graph(
  rmw_gid_t subscription_gid, const std::string & name, const std::string & namespace_,
  publish_callback_t publish_callback)
{
  std::lock_guard<std::mutex> guard(node_update_mutex_new);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.associate_reader(
    subscription_gid, gid, name, namespace_);

  rmw_ret_t rmw_ret = publish_callback(pub, static_cast<void *>(&msg));

  if (RMW_RET_OK != rmw_ret) {
    static_cast<void>(graph_cache.dissociate_reader(
      subscription_gid, gid, name, namespace_));
    return rmw_ret;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::destroy_subscriber_graph(
  rmw_gid_t subscription_gid, const std::string & name, const std::string & namespace_,
  publish_callback_t publish_callback)
{
  std::lock_guard<std::mutex> guard(node_update_mutex_new);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.dissociate_reader(
    subscription_gid, gid, name, namespace_);

  return publish_callback(pub, static_cast<void *>(&msg));
}

rmw_ret_t Context::update_publisher_graph(
  rmw_gid_t publisher_gid, const std::string & name, const std::string & namespace_,
  publish_callback_t publish_callback)
{
  std::lock_guard<std::mutex> guard(node_update_mutex_new);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.associate_writer(
    publisher_gid, gid, name, namespace_);

  rmw_ret_t rmw_ret = publish_callback(pub, static_cast<void *>(&msg));

  if (RMW_RET_OK != rmw_ret) {
    static_cast<void>(graph_cache.dissociate_writer(
      publisher_gid, gid, name, namespace_));
    return rmw_ret;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::destroy_publisher_graph(
  rmw_gid_t publisher_gid, const std::string & name, const std::string & namespace_,
  publish_callback_t publish_callback)
{
  std::lock_guard<std::mutex> guard(node_update_mutex_new);
  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.dissociate_writer(
    publisher_gid, gid, name, namespace_);

  return publish_callback(pub, static_cast<void *>(&msg));
}

rmw_ret_t Context::update_client_graph(
  rmw_gid_t request_publisher_gid, rmw_gid_t response_subscriber_gid,
  const std::string & name, const std::string & namespace_,
  publish_callback_t publish_callback)
{
  std::lock_guard<std::mutex> guard(node_update_mutex_new);
  graph_cache.associate_writer(
    request_publisher_gid, gid, name, namespace_);

  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.associate_reader(
    response_subscriber_gid, gid, name, namespace_);

  rmw_ret_t rmw_ret = publish_callback(pub, static_cast<void *>(&msg));

  if (RMW_RET_OK != rmw_ret) {
    static_cast<void>(graph_cache.dissociate_reader(
      response_subscriber_gid, gid, name, namespace_));
    static_cast<void>(graph_cache.dissociate_writer(
      request_publisher_gid, gid, name, namespace_));
    return rmw_ret;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::destroy_client_graph(
  rmw_gid_t request_publisher_gid, rmw_gid_t response_subscriber_gid,
  const std::string & name, const std::string & namespace_,
  publish_callback_t publish_callback)
{
  std::lock_guard<std::mutex> guard(node_update_mutex_new);
  graph_cache.dissociate_writer(
    request_publisher_gid, gid, name, namespace_);

  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.dissociate_reader(
    response_subscriber_gid, gid, name, namespace_);

  return publish_callback(pub, static_cast<void *>(&msg));
}

rmw_ret_t Context::update_service_graph(
  rmw_gid_t request_subscriber_gid, rmw_gid_t response_publisher_gid,
  const std::string & name, const std::string & namespace_,
  publish_callback_t publish_callback)
{
  std::lock_guard<std::mutex> guard(node_update_mutex_new);
  graph_cache.associate_reader(
    request_subscriber_gid, gid, name, namespace_);

  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.associate_writer(
    response_publisher_gid, gid, name, namespace_);

  rmw_ret_t rmw_ret = publish_callback(pub, static_cast<void *>(&msg));

  if (RMW_RET_OK != rmw_ret) {
    static_cast<void>(graph_cache.dissociate_writer(
      response_publisher_gid, gid, name, namespace_));
    static_cast<void>(graph_cache.dissociate_reader(
      request_subscriber_gid, gid, name, namespace_));
    return rmw_ret;
  }

  return RMW_RET_OK;
}

rmw_ret_t Context::destroy_service_graph(
  rmw_gid_t request_subscriber_gid, rmw_gid_t response_publisher_gid,
  const std::string & name, const std::string & namespace_,
  publish_callback_t publish_callback)
{
  std::lock_guard<std::mutex> guard(node_update_mutex_new);
  graph_cache.dissociate_reader(
    request_subscriber_gid, gid, name, namespace_);

  rmw_dds_common::msg::ParticipantEntitiesInfo msg =
    graph_cache.dissociate_writer(
    response_publisher_gid, gid, name, namespace_);

  return publish_callback(pub, static_cast<void *>(&msg));
}

}  // namespace rmw_dds_common
