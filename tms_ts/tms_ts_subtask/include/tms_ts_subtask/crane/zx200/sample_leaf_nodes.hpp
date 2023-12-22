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

#ifndef ZX200_SAMPLE_LEAF_NODES_HPP_
#define ZX200_SAMPLE_LEAF_NODES_HPP_

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

class LeafNodeSampleZx200Boom : public LeafNodeBase {
public:
  inline LeafNodeSampleZx200Boom(const std::string& name, const NodeConfiguration& config) : LeafNodeBase("leaf_sample_zx200_boom_sample", config, "zx200_boom", "sample_zx200_boom"){};
  inline static PortsList providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
};

class LeafNodeSampleZx200Swing : public LeafNodeBase {
public:
  inline LeafNodeSampleZx200Swing(const std::string& name, const NodeConfiguration& config) : LeafNodeBase("leaf_sample_zx200_swing_sample", config, "zx200_swing", "sample_zx200_swing"){};
  inline static PortsList providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
};

class LeafNodeSampleZx200Arm : public LeafNodeBase {
public:
  inline LeafNodeSampleZx200Arm(const std::string& name, const NodeConfiguration& config) : LeafNodeBase("leaf_sample_zx200_arm_sample", config, "zx200_arm", "sample_zx200_arm"){};
  inline static PortsList providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
};

class LeafNodeSampleZx200Bucket : public LeafNodeBase {
public:
  inline LeafNodeSampleZx200Bucket(const std::string& name, const NodeConfiguration& config) : LeafNodeBase("leaf_sample_zx200_bucket_sample", config, "zx200_bucket", "sample_zx200_bucket"){};
  inline static PortsList providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
};



#endif