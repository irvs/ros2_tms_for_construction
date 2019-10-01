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
    std::cout << "tfnode constructer" << std::endl;
    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);
    tf_broadcaster2_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);

    subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>("tf_node", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
    // tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_handle_);

  }

private:
  void topic_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg) const
  {
    //auto broadcaster = new tf2_ros::TransformBroadcaster();
    auto odom_to_footprint_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
    // tf2::Quaternion q;
    // q.setRPY(0.0, 0.0, 0.0);
    odom_to_footprint_msg->header.frame_id = "odom";
    odom_to_footprint_msg->child_frame_id = "base_footprint";
    odom_to_footprint_msg->header.stamp = rclcpp::Clock().now();
    odom_to_footprint_msg->transform.translation.x = 0;
    odom_to_footprint_msg->transform.translation.y = 0;
    odom_to_footprint_msg->transform.translation.z = 0;
    odom_to_footprint_msg->transform.rotation.x      = 0;
    odom_to_footprint_msg->transform.rotation.x      = 0;
    odom_to_footprint_msg->transform.rotation.x      = 0;
    odom_to_footprint_msg->transform.rotation.x      = 1;
    // odom_tf_.transform.translation.x = odom.pose.pose.position.x;
  // odom_tf_.transform.translation.y = odom.pose.pose.position.y;
  // odom_tf_.transform.translation.z = odom.pose.pose.position.z;
  // odom_tf_.transform.rotation      = odom.pose.pose.orientation;

  // odom_tf_.header.frame_id = FRAME_ID_OF_ODOMETRY;
  // odom_tf_.child_frame_id = CHILD_FRAME_ID_OF_ODOMETRY;
  // odom_tf_.header.stamp = now;
    // ts.header.frame_id = "map"
    // ts.child_frame_id = "odom"
    //     # ts.child_frame_id = "base_footprint"
    //     ts.header.stamp = self.get_clock().now().to_msg()

    std::cout << "topic callback" << std::endl;

    //rclcpp::Node::SharedPtr driverNode;
    //auto testmsg = std::make_shared<geometry_msgs::msg::TransformStamped>();
    this->tf_broadcaster_->sendTransform(*msg);
    this->tf_broadcaster2_->sendTransform(*odom_to_footprint_msg);
    // broadcaster.sendTransform(*msg);
    std::cout << "send!" << std::endl;
    
  }
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
  rclcpp::Node::SharedPtr node_handle_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster2_;

};

int main(int argc, char * argv[])
{
    std::cout << "tfnode start" << std::endl;
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}