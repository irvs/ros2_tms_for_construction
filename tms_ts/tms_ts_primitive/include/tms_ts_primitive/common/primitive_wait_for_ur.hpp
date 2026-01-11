//      http://www.apache.org/licenses/LICENSE-2.0
 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SAMPLE_WAIT_FOR_UR_HPP
#define SAMPLE_WAIT_FOR_UR_HPP

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
#include "tms_ts_primitive/primitive_node_base.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tms_msg_ur/action/next_step_permission.hpp"


class PrimitiveWaitForUr : public PrimitiveNodeBase
{
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;
    using NextStepPermission = tms_msg_ur::action::NextStepPermission;
    using GoalHandleWaitForUr = rclcpp_action::ClientGoalHandle<NextStepPermission>;
    PrimitiveWaitForUr();


private:
    rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
    std::map<std::pair<std::string, std::string>, double> param_from_db_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    // Member as an action client
    rclcpp_action::Client<NextStepPermission>::SharedPtr action_client_;
    std::shared_future<GoalHandleWaitForUr::SharedPtr> client_future_goal_handle_;
    //std::map<std::string, double> parameters;
    std::map<std::string, std::string> parameters;
    void goal_response_callback(const GoalHandleWaitForUr::SharedPtr& goal_handle);
    void feedback_callback(GoalHandleWaitForUr::SharedPtr,
                            const std::shared_ptr<const NextStepPermission::Feedback> feedback);
    void result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                        const GoalHandleWaitForUr::WrappedResult& result);
};

#endif