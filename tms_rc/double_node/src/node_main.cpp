#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

// #include "turtlebot3_msgs/msg/sensor_state.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "sensor_msgs/msg/imu.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "constants.h"
// #include "joint_state.h"
// #include "lidar.h"
#include "odometry.h"

namespace doublenode
{
class DoubleNode : public rclcpp::Node
{
 public:
  explicit DoubleNode(const std::string &node_name)
   : Node(node_name)
  {
    RCLCPP_INFO(get_logger(), "Init DoubleNode Main");

    node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

    // joint_state_ = std::make_shared<JointState>();
    // lidar_ = std::make_shared<Lidar>();
    odom_ = std::make_shared<Odometry>();
    // odom_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_handle_);

    // rclcpp::Clock::SharedPtr clock = node_handle_->get_clock();
    // guidebotListener guidebotListener(clock);
    // tfBuffer_ = std::make_shared<tf2_ros::Buffer>(node_handle_->get_clock());
    // tf2_ros::Buffer tfBuffer_;
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);

    

    // joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(JointStateTopic, rmw_qos_profile_default);
    // laser_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(ScanTopic, rmw_qos_profile_default);

    auto odom_to_footprint_callback = 
      [this](const geometry_msgs::msg::Vector3::SharedPtr odom) -> void
      {
        // this->input_odom_stack_ = odom;
        // std::cout << odom->pose.pose.position.x << std::endl;
        // std::cout << odom->pose.pose.orientation.x << std::endl;
        // std::cout << odom->twist.twist.linear.x << std::endl;
        // geometry_msgs::msg::PoseStamped odom_pose;
        // geometry_msgs::msg::PoseStamped ident;
        // ident.header.frame_id = nav2_util::strip_leading_slash(f);
        // ident.header.stamp = this->now();
        // tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

        // try {
        //   tfBuffer->transform(ident, odom_pose, "odom");
        // } catch (tf2::TransformException e) {
        //   pass;
        //   }

        // std::cout << tfBuffer_.allFramesAsString()<<std::endl;
      

      // std::cout << traslation << std::endl;
        
        this->odom_pub_->publish(this->odom_->getOdom(this->now(), WheelRadius, odom));
        this->tf_broadcaster_->sendTransform(this->odom_->getOdomTf());
      };

    odom_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>("/odom", odom_to_footprint_callback);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/wheel", rmw_qos_profile_default);
    // input_odom_stack_ = nav_msgs::msg::Odometry();
    // time_pub_ = this->create_publisher<builtin_interfaces::msg::Time>(TimeTopic, rmw_qos_profile_default);

    // auto sensor_state_callback = 
    //   [this](const turtlebot3_msgs::msg::SensorState::SharedPtr sensor_state) -> void
    //   {
    //     this->joint_state_->updateRadianFromTick(sensor_state);
    //   };

    // sensor_state_sub_ = this->create_subscription<turtlebot3_msgs::msg::SensorState>(SensorStateTopic, sensor_state_callback);

    // auto laser_scan_callback = 
    //   [this](const sensor_msgs::msg::LaserScan::SharedPtr laser_scan) -> void
    //   {
    //     this->lidar_->makeFullRange(laser_scan);
    //   };

    // laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(ScanHalfTopic, laser_scan_callback);

    // auto imu_callback = 
    //   [this](const sensor_msgs::msg::Imu::SharedPtr imu) -> void
    //   {
    //     this->odom_->updateImu(imu);
    //   };

    // imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(ImuTopic, imu_callback);

    // joint_state_timer_ = this->create_wall_timer(
    //   JointStatePublishPeriodMillis,
    //   [this]()
    //   {
    //     this->joint_state_pub_->publish(this->joint_state_->getJointState(this->now()));
    //   }
    // );

    // laser_scan_timer_ = this->create_wall_timer(
    //   ScanPublishPeriodMillis,
    //   [this]()
    //   {
    //     this->laser_scan_pub_->publish(this->lidar_->getLaserScan(this->now()));
    //   }
    // );

    // odom_timer_ = this->create_wall_timer(
    //   OdometryPublishPeriodMillis,
    //   [this]()
    //   {
    //     // this->odom_->updateJointState(this->joint_state_->getJointState(this->now()));
    //     this->odom_pub_->publish(this->odom_->getOdom(this->now(), WheelRadius, this->input_odom_stack_));
    //     this->tf_broadcaster_->sendTransform(this->odom_->getOdomTf());
    //   }
    // );

    // time_timer_ = this->create_wall_timer(
    //   TimeSyncPublishPeriodMillis,
    //   [this]()
    //   {
    //     auto time_msg = builtin_interfaces::msg::Time();
    //     time_msg = this->now();
    //     this->time_pub_->publish(time_msg);
    //   }
    // );
  }

  virtual ~DoubleNode(){};

 private:
  rclcpp::Node::SharedPtr node_handle_;

  // std::shared_ptr<JointState> joint_state_;
  // std::shared_ptr<Lidar> lidar_;
  std::shared_ptr<Odometry> odom_;
  // std::shared_ptr<geometry_msgs::msg::PoseStamped> odom_pose;

  
  // nav_msgs::msg::Odometry::SharedPtr input_odom_stack_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


  // rclcpp::Subscription<turtlebot3_msgs::msg::SensorState>::SharedPtr sensor_state_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr odom_sub_;

  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
  // rclcpp::Publisher<builtin_interfaces::msg::Time>::SharedPtr time_pub_;

  // rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  // rclcpp::TimerBase::SharedPtr laser_scan_timer_;
  // rclcpp::TimerBase::SharedPtr time_timer_;
};
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<doublenode::DoubleNode>("double_node");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}