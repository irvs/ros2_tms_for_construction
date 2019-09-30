#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>


using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("tf_node", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
    // tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_handle_);

  }

private:
  void topic_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
    {
      //auto broadcaster = new tf2_ros::TransformBroadcaster();
      rclcpp::Node::SharedPtr driverNode;
      tf2_ros::StaticTransformBroadcaster broadcaster(driverNode);
      //auto testmsg = std::make_shared<geometry_msgs::msg::TransformStamped>();
      broadcaster.sendTransform(*msg);
      
    }
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
    // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}