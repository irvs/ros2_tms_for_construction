#include "tms_ts_primitive/Bulldozer/primitive_bulldozer_blade_control.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

PrimitiveBulldozerBladeControl::PrimitiveBulldozerBladeControl()
: PrimitiveNodeBase("st_bulldozer_blade_control_node")
{
  action_server_ = rclcpp_action::create_server<tms_msg_ts::action::LeafNodeBase>(
    this,
    "st_bulldozer_blade_control",
    std::bind(&PrimitiveBulldozerBladeControl::handle_goal, this, _1, _2),
    std::bind(&PrimitiveBulldozerBladeControl::handle_cancel, this, _1),
    std::bind(&PrimitiveBulldozerBladeControl::handle_accepted, this, _1));

  // Client action name (as requested)
  action_client_ = rclcpp_action::create_client<BladeAction>(this, "tms_rp_set_bulldozer_blade");
}

bool PrimitiveBulldozerBladeControl::get_required_(const std::string & key, size_t idx, double & out) const
{
  const auto it = parameters_.find({key, std::to_string(idx)});
  if (it == parameters_.end()) {
    return false;
  }
  out = it->second;
  return true;
}

rclcpp_action::GoalResponse PrimitiveBulldozerBladeControl::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal)
{
  parameters_ =
    CustomGetParamFromDB<std::pair<std::string, std::string>, double>(goal->model_name, goal->record_name);

  if (parameters_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameters from DB");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // joint_name_ must be non-empty
  if (joint_name_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "joint_name_ is empty; cannot construct blade goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // goal_position[i] must exist for i=0..n-1 (NO zero-fill)
  const size_t n = joint_name_.size();
  for (size_t i = 0; i < n; ++i) {
    double tmp = 0.0;
    if (!get_required_("goal_position", i, tmp)) {
      RCLCPP_ERROR(this->get_logger(),
        "DB missing required key: (goal_position, %zu) for joint '%s'",
        i, joint_name_[i].c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PrimitiveBulldozerBladeControl::handle_cancel(
  const std::shared_ptr<ServerGoalHandle> /*server_goal_handle*/)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel primitive node");

  try {
    if (client_future_goal_handle_.valid() &&
        client_future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      auto client_goal_handle = client_future_goal_handle_.get();
      if (client_goal_handle) {
        action_client_->async_cancel_goal(client_goal_handle);
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in cancel relay: %s", e.what());
  }

  return rclcpp_action::CancelResponse::ACCEPT;
}

void PrimitiveBulldozerBladeControl::handle_accepted(const std::shared_ptr<ServerGoalHandle> goal_handle)
{
  std::thread{std::bind(&PrimitiveBulldozerBladeControl::execute, this, _1), goal_handle}.detach();
}

void PrimitiveBulldozerBladeControl::execute(const std::shared_ptr<ServerGoalHandle> server_goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "primitive(st_bulldozer_blade_control) is executing...");

  auto leaf_result = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();
  leaf_result->result = false;

  // Downstream action server availability
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Downstream action server (tms_rp_set_bulldozer_blade) not available");
    if (server_goal_handle->is_active()) {
      server_goal_handle->abort(leaf_result);
    }
    return;
  }

  // Build goal (control_type fixed to 0 as requested)
  BladeAction::Goal goal_msg{};
  goal_msg.joint_name = joint_name_;
  goal_msg.control_type = 0;

  const size_t n = joint_name_.size();
  goal_msg.goal_position.resize(n);

  for (size_t i = 0; i < n; ++i) {
    double v = 0.0;
    if (!get_required_("goal_position", i, v)) {
      RCLCPP_ERROR(this->get_logger(),
        "DB missing required key at execute(): (goal_position, %zu) for joint '%s'",
        i, joint_name_[i].c_str());

      if (server_goal_handle->is_active()) {
        server_goal_handle->abort(leaf_result);
      }
      return;
    }
    goal_msg.goal_position[i] = v;
  }

  // control_type=0 => do not use velocity/effort; keep empty (no zero-fill)
  goal_msg.velocity.clear();
  goal_msg.effort.clear();

  typename Client::SendGoalOptions opt;

  opt.goal_response_callback =
    [this](const typename ClientGoalHandle::SharedPtr & gh) {
      this->goal_response_callback(gh);
    };

  opt.feedback_callback =
    [this, server_goal_handle](typename ClientGoalHandle::SharedPtr /*gh*/,
                               const std::shared_ptr<const BladeAction::Feedback> feedback)
    {
      // LeafNodeBase feedback is not defined here; if needed, map and publish.
      // For now: optional log (avoid spam by throttling if you want).
      (void)server_goal_handle;
      (void)feedback;
    };

  opt.result_callback =
    [this, server_goal_handle](const WrappedResult & res) {
      this->result_callback(server_goal_handle, res);
    };

  RCLCPP_INFO(this->get_logger(), "Sending blade goal to tms_rp_set_bulldozer_blade");
  client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, opt);
}

void PrimitiveBulldozerBladeControl::goal_response_callback(const typename ClientGoalHandle::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Blade goal was rejected by downstream server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Blade goal accepted by downstream server, waiting for result");
  }
}

void PrimitiveBulldozerBladeControl::result_callback(
  const std::shared_ptr<ServerGoalHandle> server_goal_handle,
  const WrappedResult & result)
{
  if (!server_goal_handle->is_active()) {
    RCLCPP_WARN(this->get_logger(), "Attempted to finish an inactive server goal");
    return;
  }

  auto result_to_leaf = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();

  const bool success_flag =
    (result.code == rclcpp_action::ResultCode::SUCCEEDED) &&
    (result.result ? result.result->success : true);

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      result_to_leaf->result = success_flag;
      if (success_flag) {
        server_goal_handle->succeed(result_to_leaf);
      } else {
        server_goal_handle->abort(result_to_leaf);
      }
      break;

    case rclcpp_action::ResultCode::ABORTED:
      result_to_leaf->result = false;
      server_goal_handle->abort(result_to_leaf);
      break;

    case rclcpp_action::ResultCode::CANCELED:
      result_to_leaf->result = false;
      server_goal_handle->canceled(result_to_leaf);
      break;

    default:
      result_to_leaf->result = false;
      server_goal_handle->abort(result_to_leaf);
      break;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrimitiveBulldozerBladeControl>());
  rclcpp::shutdown();
  return 0;
}
