#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
using namespace std::chrono_literals;
using std::placeholders::_1;


class OdomTfNode : public rclcpp::Node
{
public:
  OdomTfNode()
  : Node("odom_tf_node")
  {
    std::cout << "tfnode constructer" << std::endl;
    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/wheel", 10, std::bind(&OdomTfNode::topic_callback, this, _1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    
    auto odom_to_footprint_msg = std::make_shared<geometry_msgs::msg::TransformStamped>();
    odom_to_footprint_msg->header.stamp = rclcpp::Clock().now();
    odom_to_footprint_msg->header.frame_id = "odom";
    odom_to_footprint_msg->child_frame_id = "base_footprint";
    odom_to_footprint_msg->transform.translation.x = msg->pose.pose.position.x;
    odom_to_footprint_msg->transform.translation.y = msg->pose.pose.position.y;
    odom_to_footprint_msg->transform.translation.z = msg->pose.pose.position.z;
    odom_to_footprint_msg->transform.rotation = msg->pose.pose.orientation;

    //auto t = odom_to_footprint_msg->transform.translation;
    //auto r = odom_to_footprint_msg-> transform.rotation;
    //std::cout << "publish {x:" << t.x << " y:" << t.y << " z:" << t.z << std::endl;
    //std::cout << "        {x:" << r.x << " y:" << r.y << " z:" << r.z << " w:" << r.w << std::endl;

    this->tf_broadcaster_->sendTransform(*odom_to_footprint_msg);
    
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Node::SharedPtr node_handle_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
};

int main(int argc, char * argv[])
{
    std::cout << "tfnode start" << std::endl;
    rclcpp::init(argc, argv);
    
    rclcpp::spin(std::make_shared<OdomTfNode>());
    rclcpp::shutdown();
    return 0;
}