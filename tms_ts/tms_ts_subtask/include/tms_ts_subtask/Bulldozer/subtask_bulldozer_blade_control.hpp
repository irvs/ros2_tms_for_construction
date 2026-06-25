#ifndef TMS_TS_SUBTASK_BULLDOZER_BLADE_CONTROL_HPP
#define TMS_TS_SUBTASK_BULLDOZER_BLADE_CONTROL_HPP

#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tms_msg_ts/action/leaf_node_base.hpp"
#include "tms_msg_rp/action/tms_rp_bulldozer_blade.hpp"

#include "tms_ts_subtask/subtask_node_base.hpp"

class SubtaskBulldozerBladeControl : public SubtaskNodeBase
{
public:
  using BladeAction = tms_msg_rp::action::TmsRpBulldozerBlade;

  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<BladeAction>;
  using Client = rclcpp_action::Client<BladeAction>;
  using WrappedResult = Client::WrappedResult;

  SubtaskBulldozerBladeControl();

private:
  // --- server (LeafNodeBase) ---
  rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<ServerGoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle);
  void execute(const std::shared_ptr<ServerGoalHandle> goal_handle);

  // --- client (BladeAction) ---
  Client::SharedPtr action_client_;
  std::shared_future<typename ClientGoalHandle::SharedPtr> client_future_goal_handle_;

  void goal_response_callback(const typename ClientGoalHandle::SharedPtr & goal_handle);

  void result_callback(
    const std::shared_ptr<ServerGoalHandle> server_goal_handle,
    const WrappedResult & result);

  // DB parameters: ("key","index") -> value
  std::map<std::pair<std::string, std::string>, double> parameters_;

  // Blade joints (ordered). You can replace this with a runtime-configured list if needed.
  std::vector<std::string> joint_name_{"lift_joint", "tilt_joint", "angle_joint"};

  // Fetch required value from DB with index; return false if missing (NO zero-fill).
  bool get_required_(const std::string & key, size_t idx, double & out) const;
};

#endif  // TMS_TS_SUBTASK_BULLDOZER_BLADE_CONTROL_HPP
