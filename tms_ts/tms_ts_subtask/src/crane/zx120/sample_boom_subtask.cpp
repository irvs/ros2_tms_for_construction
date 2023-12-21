
#include <memory>
#include <thread>
#include "std_msgs/msg/float64.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tms_msg_ts/action/leaf_node_base.hpp"
#include "tms_ts_subtask/subtask_node_base.hpp"

#include <glog/logging.h>


class Zx120SampleBoomActionServer : public BaseClassSubtasks
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;


  Zx120SampleBoomActionServer(): BaseClassSubtasks("zx120_sample_boom_action_server")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Float64>("/zx120/boom/cmd",10);
    this->action_server_ = rclcpp_action::create_server<tms_msg_ts::action::LeafNodeBase>(
      this,
      "sample_zx120_boom",
      std::bind(&Zx120SampleBoomActionServer::handle_goal, this,std::placeholders::_1, std::placeholders::_2),
      std::bind(&Zx120SampleBoomActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&Zx120SampleBoomActionServer::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std::map<std::string, int> parameters;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with parameter_id = %s", goal->parameter_id.c_str());
    parameters = GetParamFromDB(goal->parameter_id);
    RCLCPP_INFO(this->get_logger(), "param = %d", parameters["zx120_boom_goal_pos"]);
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
    std::thread{std::bind(&Zx120SampleBoomActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "subtask is executing...");
    rclcpp::Rate loop_rate(1);
    const auto goal_pos = parameters["zx120_boom_goal_pos"];
    RCLCPP_INFO(this->get_logger(), "param2 = %d", parameters["zx120_boom_goal_pos"]);
    auto feedback = std::make_shared<tms_msg_ts::action::LeafNodeBase::Feedback>();
    auto & current_pos = feedback->current_pos;
    auto result = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();
    int deg = 0;
    std_msgs::msg::Float64 msg_rad;

    while(deg >= goal_pos){
        if (goal_handle->is_canceling()) {
            result->result = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "subtask execution is canceled");
            return;
        }
        deg += goal_pos / float(20.0);
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
	// Initialize Google's logging library.
	google::InitGoogleLogging(argv[0]);
	google::InstallFailureSignalHandler();
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Zx120SampleBoomActionServer>());
  rclcpp::shutdown();
  return 0;
}




