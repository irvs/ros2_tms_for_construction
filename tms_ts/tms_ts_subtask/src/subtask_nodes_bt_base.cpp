#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tms_msg_ts/action/ts_do_subtask.hpp"
#include "nlohmann/json.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class SubtaskNodeBase : public rclcpp::Node{
public:
using TsDoSubtask= tms_msg_ts::action::TsDoSubtask;
using GoalHandleTsDoSubtask = rclcpp_action::ServerGoalHandle<TsDoSubtask>;
// コンストラクタの範囲を調べてSubtaskNodeBase::node_name(), SubtaskNodeBase::id()の実装を変更すること

    explicit SubtaskNodeBase(): Node(SubtaskNodeBase::node_name()){
        _dict = init_argument();
        cb_group_ = std::make_shared<rclcpp::CallbackGroup>(rclcpp::CallbackGroupType::Reentrant);
        this->_action_server_ = rclcpp_action::create_server<TsDoSubtask>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            std::string("subtask_node_" + SubtaskNodeBase::id()),
            std::bind(&SubtaskNodeBase::goal_callback, this, _1, _2),  // GoalCallback
            std::bind(&SubtaskNodeBase::cancel_callback, this, _1),     // CancelCallback
            std::bind(&SubtaskNodeBase::handle_accepted, this, _1));   // ExecuteCallback
    }

private:
    std::unordered_map<std::string, std::string> _dict;
    rclcpp_action::Server<TsDoSubtask>::SharedPtr _action_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::TimerBase::SharedPtr _timer_;

    void destroy(){
        _action_server_.reset();
        rclcpp::shutdown();
    }

    rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID & uuid, 
        std::shared_ptr<const TsDoSubtask::Goal> goal){
        RCLCPP_INFO(get_logger(), "Received goal request");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<GoalHandleTsDoSubtask> goal_handle){
        RCLCPP_INFO(get_logger(), "Received cancel request");
        _timer_ = create_wall_timer(0s, std::bind(&SubtaskNodeBase::_cancel_service_callback, this));
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleTsDoSubtask> goal_handle){
        RCLCPP_INFO(get_logger(), ">> Start");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<TsDoSubtask::Feedback>();
        _dict = nlohmann::json::parse(goal_handle->get_goal()->arg_json);
        auto result = std::make_shared<TsDoSubtask::Result>();
        result = service_callback(_dict);
        if (goal_handle->is_canceling()){
            goal_handle->canceled(result);
            result->message = "Canceled";
        }else if (result->message == "Success"){
            goal_handle->succeed(result);
        }else if (result->message == "Canceled"){
            goal_handle->canceled(result);
        }else{
            //goal_handle->abort(result);
            goal_handle->publish_feedback(feedback);
        }
        if (result->message != "Success"){ RCLCPP_WARN(get_logger(), "return %s", result->message.c_str()); }
    }

    virtual std::string node_name(){ return "default_node"; };
    virtual int id(){ return 0; };
    virtual std::shared_ptr<TsDoSubtask::Result> service_callback(const std::unordered_map<std::string, std::string> &){
        return std::make_shared<TsDoSubtask::Result>();
    }

    void _cancel_service_callback(){
        if (_timer_){ _timer_->cancel(); }
        cancel_service_callback();
    }

    virtual void cancel_service_callback() {}

    std::unordered_map<std::string, std::string> init_argument(){
        return std::unordered_map<std::string, std::string>();
    }    
};