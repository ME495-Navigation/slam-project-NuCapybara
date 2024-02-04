/// PARAMETERS:
///     rate (double): frequency of the timer, in Hz
///     x0 (double): starting x location of the turtlebot (m)
///     y0 (double): starting y location of the turtlebot (m)
///     theta0 (double): starting theta location of the turtlebot (rad)
///     obstacles/x (double[]): list of x coordinates of cylindrical obstacles (m)
///     obstacles/y (double[]): list of r coordinates of cylindrical obstacles (m)
///     obstacles/r (double): radius of cylindrical obstacles (m)
///     arena_x_length : X length of rectangular arena (m)
///     arena_y_length : Y length of rectangular arena (m)
/// PUBLISHES:
///     ~/timestep (std_msgs::msg::Uint64): current timestep of simulation
///     ~/obstacles (visualization_msgs::msg::MarkerArray): marker objects representing cylinders
///     ~/walls (visualization_msgs::msg::MarkerArray): marker objects representing walls of arena
/// SERVERS:
///     ~/reset (std_srvs::srv::Empty): resets the simulation to the initial state
///     ~/teleport (nusim::srv::Teleport): teleports the turtle to a given x, y, theta value
/// CLIENTS:
///     none
/// BROADCASTS:
///    nusim/world -> red/base_footprint


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nusim/srv/Teleport.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtlebot_msgs/msg/WheelCommands.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlelib/include/turtlelib/se2d.hpp"
#include "turtlelib/include/turtlelib/diff_drive.hpp"
#include "turtlelib/include/turtlelib/geometry2d.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/SensorData.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nuturtle_control/srv/Initial_pose.hpp"


using namespace std::chrono_literals;
using namespace turtlelib;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class odometry: public rclcpp::Node
{
public:
  Nusim()
  : Node("odometry"), timestep_(0)
  {

    turtlelib::DiffDrive temp(track_width, wheel_radius);
    robot = temp;


    ///parameter delcaration
    declare_parameter("body_id", body_id);
    body_id = get_parameter("body_id").as_string();
    if(body_id.empty()){
        RCLCPP_DEBUG_STREAM(node->get_logger(), "BodyID not specified" << 4);
    }
    
    declare_parameter("odom_id", "odom");
    odom_id = get_parameter("odom_id").as_string();
    RCLCPP_INFO_STREAM(get_logger(), "Odom ID: " << odom_id);

    declare_parameter("wheel_left", body_id);
    wheel_left = get_parameter("wheel_left").as_string();
    if(wheel_left.empty()){
        RCLCPP_DEBUG_STREAM(node->get_logger(), "left wheel joint name not specified" << 4);
    }

    declare_parameter("wheel_right", body_id);
    wheel_right = get_parameter("wheel_right").as_string();
    if(wheel_right.empty()){
        RCLCPP_DEBUG_STREAM(node->get_logger(), "right wheel joint name not specified" << 4);
    }

    //odom id defination
    odom.header.frame_id = odom_id; //relatively world id?
    odom.child_frame_id = body_id;

    /// joint state subscriber and odom publisher
    joint_state_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states",10, std::bind(&MinimalSubscriber::js_callback, this, _1));
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);


    br = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    ///
    initial_pose_=create_service<nuturtle_control::srv::Initial_pose>("initial_pose",
    std::bind(&Odometry::initial_pose, this, std::placeholders::_1, std::placeholders::_2));
  }

private:

    void js_callback(const sensor_msgs::msg::JointState & js){
        ///Frame this joint state is associated with
        string js_frameid = js.header.frame_id;
        const auto time_stamp = js.header.stamp;
        const auto joint_name_list = js.name;
        const auto joint_position_list = js.position;
        const auto joint_velocity_list = js.velocity;

        auto left_wheel_ptr = find(joint_name_list.begin(), joint_name_list.end(), wheel_left);
        auto right_wheel_ptr = find(joint_name_list.begin(), joint_name_list.end(), wheel_right);    

        if(first){
            first = false;
        }
        else{
            //not first time, gonna pair with last_joint_state
            // If left wheel was found 
            if (left_wheel_ptr != joint_name_list.end() && right_wheel_ptr != joint_name_list.end())  
            {   // calculating the index 
                int left_index = left_wheel_ptr - joint_name_list.begin(); 
                int right_index = right_wheel_ptr - joint_name_list.begin();
                double left_wheel_angle = joint_position_list[left_index];
                double right_wheel_angle = joint_position_list[right_index];
                double left_wheel_velocity = joint_velocity_list[left_index];
                double right_wheel_velocity = joint_velocity_list[right_index];

                const auto dl = left_wheel_angle - last_joint_state.position[left_index];
                const auto dr = right_wheel_angle - last_joint_state.position[right_index];
                const auto currTime = js.header.stamp.sec 4   + 1e-9 * js.header.stamp.nanosec;
                const auto lastTime = last_joint_state.header.stamp.sec + 1e-9 * last_joint_state.header.stamp.nanosec;
                const auto dt = currTime - lastTime;

                const auto prev_x = robot.get_current_config().translation().x;
                const auto prev_y = robot.get_current_config().translation().y;
                const auto prev_phi = robot.get_current_config().rotation();

                // update robot config and everything
                robot.forwardKinematics(WheelState{dl, dr});

                ///ODOM POSE PART
                ///define odom pose/pose
                odom.header.stamp = time_stamp;
                odom.pose.pose.position.x = robot.get_current_config().translation().x;
                odom.pose.pose.position.y = robot.get_current_config().translation().y;
                ///define odom pose/orientation(quaternion)
                tf2::Quaternion q;
                q.setRPY(0, 0, robot.get_current_config().rotation());
                odom.pose.pose.orientation.x = q.x();
                odom.pose.pose.orientation.y = q.y();
                odom.pose.pose.orientation.z = q.z();
                odom.pose.pose.orientation.w = q.w();
                ///define odom covariance
                odom.pose.covariance = [0]; ///do we really need to?

                ///ODOM VELOCITY PART
                ///define odom twist/twist
                odom.twist.twist.linear.x = (robot.get_current_config().translation().x - prev_x) / dt;
                odom.twist.twist.linear.y = (robot.get_current_config().translation().y - prev_y) / dt;
                odom.twist.twist.angular.z = normalize_angle(robot.get_current_config().rotation() - prev_phi) / dt;

                ///define odom twist/covariance
                odom.twist.covariance = [0]; ///do we really need to?

                ///publish odom
                odom_publisher_->publish(odom);

                ///TF BROADCAST
                geometry_msgs::TransformStamped transformStamped;
                
                transformStamped.header.stamp = js.header.stamp;
                transformStamped.header.frame_id = odom_id;
                transformStamped.child_frame_id = body_id;
                transformStamped.transform.translation.x = robot.get_current_config().translation().x;
                transformStamped.transform.translation.y = robot.get_current_config().translation().y;
                transformStamped.transform.rotation.x = q.x();
                transformStamped.transform.rotation.y = q.y();
                transformStamped.transform.rotation.z = q.z();
                transformStamped.transform.rotation.w = q.w();
                br.sendTransform(transformStamped);
            }
       
            else{
                RCLCPP_DEBUG_STREAM(node->get_logger(), "Wheel ID not found" << 4);
            } 
        }
        last_joint_state = js;

    }

    void initial_pose(
    std::shared_ptr<nuturtle_control::srv::Initial_pose::Request> request,
    std::shared_ptr<nuturtle_control::srv::Initial_pose::Response> response){
        turtlelib::Transform2D currConfig(turtlelib::Vector2D{request->x, request->y}, request->theta);
        robot = turtlelib::DiffDrive(currConfig, wheel_radius, track_width);
        response->reset = true;
    }
    
    
    
    std::string wheel_left, wheel_right, body_id, odom_id;
    double wheel_radius, track_width;
    sensor_msgs::msg::JointState last_joint_state;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription__;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_;
    turtlelib::DiffDrive robot;
    nav_msgs::msg::Odometry odom;
    std::unique_ptr<tf2_ros::TransformBroadcaster> br;
    bool first = true;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_control>());
  rclcpp::shutdown();
  return 0;
}
