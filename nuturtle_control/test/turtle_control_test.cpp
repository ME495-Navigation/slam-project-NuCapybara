// Copyright 2023 Nick Morales.
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

/// @file This is an example ROS 2 node that checks assertions using Catch2.
/// It simply checks if the "test_service" service is available at least once
/// during the duration of the test.

#include "catch_ros2/catch_ros2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <cmath>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"


using namespace std::chrono_literals;


turtlelib::DiffDrive robot{0.033, 0.16};
geometry_msgs::msg::Twist cmd;
void test_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands msg) {
  RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "I heard Wheel Command as (WC: left velocity, right velocity): " << msg.left_velocity << " " << msg.right_velocity);
  turtlelib::Twist2D twist{cmd.angular.z, cmd.linear.x, 0.0};

  double left_velocity_calculated = robot.inverseKinematics(twist).l / 0.024;
  double right_velocity_calculated = robot.inverseKinematics(twist).r / 0.024;
  RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "left velocity calculated, right velocity calculated): " << left_velocity_calculated << " " << right_velocity_calculated);
  REQUIRE(msg.left_velocity == left_velocity_calculated);
  REQUIRE(msg.right_velocity == right_velocity_calculated);
}


TEST_CASE("cmd_vel test", "turtle_control_test") {
  auto node = rclcpp::Node::make_shared("turtle_control_test");
  /// cmd_vel publisher
  /// wheel_cmd_subscriber
  auto cmd_vel_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  auto wheel_cmd_subscriber_ = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "wheel_cmd", 10, test_cmd_callback);
  
  // Declare a parameter on the node
  // (the default catch_ros2 node main will allow ROS arguments
  // like parameters to be passed to nodes in test files)
  node->declare_parameter<double>("test_duration", 5.0);

  // Get value of the parameter
  // This line will cause a runtime error if a value
  // for the "test_duration" parameter is not passed to the node
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  rclcpp::Time start_time = rclcpp::Clock().now();

  cmd.linear.x = 0.1;
  cmd.angular.z = 0.1;
  cmd_vel_publisher_->publish(cmd);
  RCLCPP_INFO_STREAM(rclcpp::get_logger("turtle_control_test"), "I published cmd_vel");


  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    // Repeatedly check for the dummy service until its found
    // if (client->wait_for_service(0s)) {
    //   service_found = true;
    //   break;
    // }

    rclcpp::spin_some(node);
  }

  // Test assertions - check that the dummy node was found
  // CHECK(service_found);

}