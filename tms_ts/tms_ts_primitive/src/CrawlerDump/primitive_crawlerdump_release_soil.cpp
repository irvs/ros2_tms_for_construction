#include <thread>
#include <map>
#include <string>
#include <memory>
#include <chrono>

#include "tms_ts_primitive/CrawlerDump/primitive_crawlerdump_release_soil.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace
{
inline double get_or0(const std::map<std::string, double> & m, const std::string & k)
{
  auto it = m.find(k);
  return (it == m.end()) ? 0.0 : it->second;  // 副作用なし（キー追加しない）
}
}  // namespace

PrimitiveCrawlerDumpReleaseSoil::PrimitiveCrawlerDumpReleaseSoil()
: PrimitiveNodeBase("primitive_crawlerdump_release_soil_node")
{
  action_server_ = rclcpp_action::create_server<tms_msg_ts::action::LeafNodeBase>(
    this,
    "primitive_crawlerdump_release_soil",
    std::bind(&PrimitiveCrawlerDumpReleaseSoil::handle_goal, this, _1, _2),
    std::bind(&PrimitiveCrawlerDumpReleaseSoil::handle_cancel, this, _1),
    std::bind(&PrimitiveCrawlerDumpReleaseSoil::handle_accepted, this, _1));

  action_client_ = rclcpp_action::create_client<TmsRpCrawlerDumpDumpAngle>(this, "tms_rp_set_dump_angle");
}

rclcpp_action::GoalResponse PrimitiveCrawlerDumpReleaseSoil::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal)
{
  parameters = CustomGetParamFromDB<std::string, double>(goal->model_name, goal->record_name);
  if (parameters.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameters from DB");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PrimitiveCrawlerDumpReleaseSoil::handle_cancel(
  const std::shared_ptr<GoalHandle> /*server_goal_handle*/)
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

void PrimitiveCrawlerDumpReleaseSoil::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&PrimitiveCrawlerDumpReleaseSoil::execute, this, _1), goal_handle}.detach();
}

void PrimitiveCrawlerDumpReleaseSoil::execute(const std::shared_ptr<GoalHandle> server_goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "primitive(st_crawlerdump_release_soil) is executing...");

  auto leaf_result = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();

  // 中継先がいないなら即 abort
  if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Downstream action server (tms_rp_set_dump_angle) not available");
    leaf_result->result = false;
    if (server_goal_handle->is_active()) {
      server_goal_handle->abort(leaf_result);
    }
    return;
  }

  // --- DB -> Goal（キーが無いなら 0）---
  TmsRpCrawlerDumpDumpAngle::Goal goal_msg{};
  goal_msg.target_angle = get_or0(parameters, "target_angle");
  goal_msg.control_type = static_cast<uint8_t>(get_or0(parameters, "control_type"));
  goal_msg.velocity     = get_or0(parameters, "velocity");
  goal_msg.effort       = get_or0(parameters, "effort");

  rclcpp_action::Client<TmsRpCrawlerDumpDumpAngle>::SendGoalOptions opt;
  opt.goal_response_callback =
    [this](const GoalHandleCrawlerDumpReleaseSoil::SharedPtr & gh) { this->goal_response_callback(gh); };

  // Feedback 型は ClientGoalHandle の内側じゃなく Action::Feedback
  opt.feedback_callback =
    [this](GoalHandleCrawlerDumpReleaseSoil::SharedPtr gh,
           const std::shared_ptr<const TmsRpCrawlerDumpDumpAngle::Feedback> fb)
    {
      this->feedback_callback(gh, fb);
    };

  opt.result_callback =
    [this, server_goal_handle](const rclcpp_action::Client<TmsRpCrawlerDumpDumpAngle>::WrappedResult & res)
    {
      this->result_callback(server_goal_handle, res);
    };

  RCLCPP_INFO(this->get_logger(), "Sending goal to tms_rp_set_dump_angle");
  client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, opt);
}

void PrimitiveCrawlerDumpReleaseSoil::goal_response_callback(
  const GoalHandleCrawlerDumpReleaseSoil::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by downstream server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by downstream server, waiting for result");
  }
}

void PrimitiveCrawlerDumpReleaseSoil::feedback_callback(
  const GoalHandleCrawlerDumpReleaseSoil::SharedPtr /*gh*/,
  const std::shared_ptr<const TmsRpCrawlerDumpDumpAngle::Feedback> /*feedback*/)
{
  // 今は何もしない（必要なら LeafNodeBase の feedback へ変換して publish）
}

void PrimitiveCrawlerDumpReleaseSoil::result_callback(
  const std::shared_ptr<GoalHandle> server_goal_handle,
  const rclcpp_action::Client<TmsRpCrawlerDumpDumpAngle>::WrappedResult & result)
{
  if (!server_goal_handle->is_active()) {
    RCLCPP_WARN(this->get_logger(), "Attempted to finish an inactive server goal");
    return;
  }

  auto result_to_leaf = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();

  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      result_to_leaf->result = true;
      server_goal_handle->succeed(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Primitive execution succeeded");
      break;

    case rclcpp_action::ResultCode::ABORTED:
      result_to_leaf->result = false;
      server_goal_handle->abort(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Primitive execution aborted");
      break;

    case rclcpp_action::ResultCode::CANCELED:
      result_to_leaf->result = false;
      server_goal_handle->canceled(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Primitive execution canceled");
      break;

    default:
      result_to_leaf->result = false;
      server_goal_handle->abort(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Unknown result code");
      break;
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrimitiveCrawlerDumpReleaseSoil>());
  rclcpp::shutdown();
  return 0;
}
