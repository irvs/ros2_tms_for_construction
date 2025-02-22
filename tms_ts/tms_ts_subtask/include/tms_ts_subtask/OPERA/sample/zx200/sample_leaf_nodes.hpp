// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SAMPLE_LEAF_NODE_ZX200_HPP_
#define SAMPLE_LEAF_NODE_ZX200_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "tms_ts_subtask/leaf_node_base.hpp"

using namespace BT;
using namespace std::chrono_literals;

class LeafNodeSampleZx200 : public LeafNodeBase
{
public:
  inline LeafNodeSampleZx200(const std::string& name, const NodeConfiguration& config)
    : LeafNodeBase("leaf_node_zx200", config){};
  inline static PortsList providedPorts()
  {
    return { InputPort<float>("model_name"), InputPort<float>("record_name"), InputPort<float>("subtask_name") };
  };
};

#endif