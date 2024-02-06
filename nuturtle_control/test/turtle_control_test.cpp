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
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/src/diff_drive.hpp"
#include "turtlelib/src/geometry2d.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/SensorData.hpp"
#include "turtlelib/include/turtlelib/se2d.hpp"
#include "turtlelib/include/turtlelib/diff_drive.hpp"
#include "turtlelib/include/turtlelib/geometry2d.hpp"

using namespace std::chrono_literals;
using namespace turtlelib;
using namespace std::chrono_literals;

auto node = rclcpp::Node::make_shared("turtle_control_test");
auto wheel_cmd_subscriber_ = node->create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
  "wheel_cmd", 10, std::bind(&MinimalSubscriber::test_cmd_callback, this, _1));
auto cmd_vel_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
auto robot = turtlelib::DiffDrive robot{0.0, 0.0};

TEST_CASE("cmd_vel test", "turtle_control_test") {
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.1;
  cmd.angular.z = 0.0;
  cmd_vel_publisher_->publish(cmd);
  node->declare_parameter<double>("test_duration");

  void test_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg) {
    double left_velocity_calculated = robot.inverseKinematics(cmd).l / 0.024;
    double right_velocity_calculated = robot.inverseKinematics(cmd).r / 0.024;
    REQUIRE(msg->left_velocity == left_velocity_calculated);
    REQUIRE(msg->right_velocity == right_velocity_calculated);
  }

  rclcpp::spin(node);
}