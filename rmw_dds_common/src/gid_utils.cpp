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

#include <algorithm>
#include <cstring>

#include "rmw/types.h"

#include "rmw_dds_common/gid_utils.hpp"
#include "rmw_dds_common/msg/gid.hpp"

using rmw_dds_common::Compare_rmw_gid_t;

// (ivanpauno)
// This function could be implementation specific, though this common one should work well.
bool
Compare_rmw_gid_t::operator()(const rmw_gid_t & lhs, const rmw_gid_t & rhs) const
{
  return std::lexicographical_compare(
    lhs.data,
    lhs.data + RMW_GID_STORAGE_SIZE,
    rhs.data,
    rhs.data + RMW_GID_STORAGE_SIZE);
}

void
rmw_dds_common::convert_gid_to_msg(rmw_dds_common::msg::Gid * msg_gid, rmw_gid_t * gid)
{
  std::memcpy(&msg_gid->data, gid->data, RMW_GID_STORAGE_SIZE);
}
