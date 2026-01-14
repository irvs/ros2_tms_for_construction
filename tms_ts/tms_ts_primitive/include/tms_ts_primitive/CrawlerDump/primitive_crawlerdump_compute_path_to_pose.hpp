// ===============================
// primitive_crawlerdump_compute_path_to_pose.hpp
// ===============================

// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TMS_TS_PRIMITIVE_CRAWLERDUMP_COMPUTE_PATH_TO_POSE_HPP
#define TMS_TS_PRIMITIVE_CRAWLERDUMP_COMPUTE_PATH_TO_POSE_HPP

#include <memory>
#include <map>
#include <vector>
#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tms_msg_ts/action/leaf_node_base.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "tms_ts_primitive/primitive_node_base.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

#include <mongocxx/client.hpp>
#include <mongocxx/uri.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>

#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/array.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>

class PrimitiveCrawlerDumpComputePathToPose : public PrimitiveNodeBase
{
public:
  using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
  using GoalHandleLeaf    = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;
  using GoalHandleCompute = rclcpp_action::ClientGoalHandle<ComputePathToPose>;

  PrimitiveCrawlerDumpComputePathToPose();

private:
  rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
  rclcpp_action::Client<ComputePathToPose>::SharedPtr action_client_;
  //
  rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr publisher_;
  //
  std::shared_future<GoalHandleCompute::SharedPtr> client_future_goal_handle_;

  std::map<std::string, double> parameters_;

  mongocxx::client     mongo_client_;
  mongocxx::database   mongo_db_;
  mongocxx::collection mongo_collection_;

  bool has_param_(const std::string& k) const
  {
    return parameters_.find(k) != parameters_.end();
  }

  double get_or_(const std::string& k, double def) const
  {
    auto it = parameters_.find(k);
    return (it == parameters_.end()) ? def : it->second;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleLeaf> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleLeaf> goal_handle);
  void execute(const std::shared_ptr<GoalHandleLeaf> goal_handle);

  void goal_response_callback(const GoalHandleCompute::SharedPtr& goal_handle);

  void feedback_callback(
    GoalHandleCompute::SharedPtr,
    const std::shared_ptr<const ComputePathToPose::Feedback> feedback);

  void result_callback(
    const std::shared_ptr<GoalHandleLeaf> goal_handle,
    const GoalHandleCompute::WrappedResult& result);
    std::string output_model_name_;
    std::string output_record_name_;
};

#endif  // TMS_TS_PRIMITIVE_CRAWLERDUMP_COMPUTE_PATH_TO_POSE_HPP