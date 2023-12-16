
#include <memory>
#include <thread>
#include "std_msgs/msg/float64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tms_msg_ts/action/action_sample.hpp"


class Zx120SampleActionServer : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::ActionSample>;


  Zx120SampleActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()): Node("zx120_sample_action_server", options)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/zx120/boom/cmd",10);
    this->action_server_ = rclcpp_action::create_server<tms_msg_ts::action::ActionSample>(
      this,
      "zx120_sample",
      std::bind(&Zx120SampleActionServer::handle_goal, this,std::placeholders::_1, std::placeholders::_2),
      std::bind(&Zx120SampleActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&Zx120SampleActionServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<tms_msg_ts::action::ActionSample>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const tms_msg_ts::action::ActionSample::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->goal_pos);
    if (goal->goal_pos > 9000) { return rclcpp_action::GoalResponse::REJECT; }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&Zx120SampleActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "subtask is executing...");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<tms_msg_ts::action::ActionSample::Feedback>();
    auto & current_pos = feedback->current_pos;
    auto result = std::make_shared<tms_msg_ts::action::ActionSample::Result>();
    int deg = 0;
    std_msgs::msg::Float64 msg_rad;

    while(deg >= goal->goal_pos){
        if (goal_handle->is_canceling()) {
            result->result = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "subtask execution is canceled");
            return;
        }
        deg += goal->goal_pos / float(20.0);
        msg_rad.data = float(deg * float(M_PI / 180));
        current_pos = deg;
        goal_handle->publish_feedback(feedback); 
        publisher_->publish(msg_rad);
        sleep(1); 
    }

    if(rclcpp::ok()){
        result->result = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "subtask execution is succeeded");
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zx120SampleActionServer>());
  rclcpp::shutdown();
  return 0;
}




