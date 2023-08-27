// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// #include "pcl_ros/transforms.h"
// #include "tf2_eigen/tf2_eigen.hpp"

class TmsSpMachinePoints : public rclcpp::Node
{
public:
  TmsSpMachinePoints()
      : Node("tms_sp_machine_points")
  {
    // Declare parameters
    to_frame_ = this->declare_parameter<std::string>("to_frame", "world");

    // Declare tf2 buffer and listener
    tf_buffer_1_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_1_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_1_);
    tf_buffer_2_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_2_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_2_);

    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Declare publisher and subscriber
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("tf_machine_points", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/input/machine_points", 10, std::bind(&TmsSpMachinePoints::publish_transformed_points, this, std::placeholders::_1));
    topic_is_received = false;
  }

private:
  std::string to_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_1_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_1_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_2_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_2_;
  // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  bool topic_is_received;

  void publish_transformed_points(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Log
    if (!topic_is_received)
    {
      RCLCPP_INFO(this->get_logger(), "topic is received");
    }

    // transform merge -> map
    geometry_msgs::msg::TransformStamped transform_stamped_1;
    try
    {
      transform_stamped_1 = tf_buffer_1_->lookupTransform("map", msg->header.frame_id, rclcpp::Clock().now());
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed_msg_1;
    tf2::doTransform(*msg, transformed_msg_1, transform_stamped_1);

    // Log
    if (!topic_is_received)
    {
      RCLCPP_INFO(this->get_logger(), "transform merge -> map is done");
    }

    // transform map -> world
    geometry_msgs::msg::TransformStamped transform_stamped_2;
    try
    {
      transform_stamped_2 = tf_buffer_2_->lookupTransform(to_frame_, "map", rclcpp::Clock().now(), rclcpp::Duration::from_seconds(2.0));
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 transformed_msg_2;
    tf2::doTransform(transformed_msg_1, transformed_msg_2, transform_stamped_2);

    // Log
    if (!topic_is_received)
    {
      RCLCPP_INFO(this->get_logger(), "transform map -> world is done");
      topic_is_received = true;
    }

    publisher_->publish(transformed_msg_2);
  }

  // void publish_transformed_points2(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  // {
  //   if (!topic_is_received)
  //   {
  //     topic_is_received = true;
  //     RCLCPP_INFO(this->get_logger(), "topic is received");
  //   }

  //   // transform
  //   geometry_msgs::msg::TransformStamped transform_stamped;
  //   try
  //   {
  //     transform_stamped = tf_buffer_->lookupTransform(to_frame_, msg->header.frame_id, rclcpp::Clock().now());
  //   }
  //   catch (tf2::TransformException &ex)
  //   {
  //     RCLCPP_WARN(this->get_logger(), "%s", ex.what());
  //     return;
  //   }

  //   sensor_msgs::msg::PointCloud2 transformed_msg;
  //   Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
  //   pcl_ros::transformPointCloud(transform_matrix, *msg, transformed_msg);

  //   RCLCPP_INFO(this->get_logger(), "transform is done");

  //   publisher_->publish(transformed_msg);
  // }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TmsSpMachinePoints>());
  rclcpp::shutdown();
  return 0;
}
