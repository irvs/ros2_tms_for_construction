//      http://www.apache.org/licenses/LICENSE-2.0
 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SAMPLE_SUBTASK_MST110CR_SWING_HPP
#define SAMPLE_SUBTASK_MST110CR_SWING_HPP

#include <memory>
#include <map>

#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tms_msg_ts/action/leaf_node_base.hpp"
#include "tms_ts_subtask/subtask_node_base.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "com3_msgs/action/set_swing_angle.hpp"


class SubtaskMst110crSwing : public SubtaskNodeBase
{
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;
    using SetSwingAngle = com3_msgs::action::SetSwingAngle;
    using GoalHandleMst110crSwing = rclcpp_action::ClientGoalHandle<SetSwingAngle>;
    SubtaskMst110crSwing();


private:
    rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
    std::map<std::pair<std::string, std::string>, double> param_from_db_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    // Member as an action client
    rclcpp_action::Client<SetSwingAngle>::SharedPtr action_client_;
    std::shared_future<GoalHandleMst110crSwing::SharedPtr> client_future_goal_handle_;
    std::map<std::string, double> parameters;
    void goal_response_callback(const GoalHandleMst110crSwing::SharedPtr& goal_handle);
    void feedback_callback(GoalHandleMst110crSwing::SharedPtr,
                            const std::shared_ptr<const SetSwingAngle::Feedback> feedback);
    void result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                        const GoalHandleMst110crSwing::WrappedResult& result);
};

#endif