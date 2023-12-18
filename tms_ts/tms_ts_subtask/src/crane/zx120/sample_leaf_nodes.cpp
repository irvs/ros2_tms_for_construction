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

#include "tms_ts_subtask/crane/zx120/sample_leaf_nodes.hpp"

using namespace BT;
using namespace std::chrono_literals;

LeafNodeSampleZx120Boom::LeafNodeSampleZx120Boom(const std::string& name, const NodeConfiguration& config) : LeafNodeBase("leaf_sample_zx120_boom_sample", config){
  action_name_ = "sample_zx120_boom";
};

PortsList LeafNodeSampleZx120Boom::providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }


void LeafNodeSampleZx120Boom::on_tick(){ }