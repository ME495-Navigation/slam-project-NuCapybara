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
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

using namespace std::chrono_literals;
using namespace turtlelib;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Turtle_control : public rclcpp::Node
{
public:
  Nusim()
  : Node("turtle_control"), timestep_(0)
  {
    ///parameter delcaration
    declare_parameter("wheel_radius", 0.);
    wheel_radius = get_parameter("wheel_radius").as_double();
    if(wheel_radius == 0 || wheel_radius < 0){
        RCLCPP_DEBUG_STREAM(node->get_logger(), "Wheel_radius error" << 4);
    }

    declare_parameter("track_width", 0.);
    track_width = get_parameter("track_width").as_double();
    if(track_width == 0 || track_width < 0){
        RCLCPP_DEBUG_STREAM(node->get_logger(), "Track_width error" << 4);
    }

    declare_parameter("motor_cmd_max", 0.);
    motor_cmd_max = get_parameter("motor_cmd_max").as_double();
    if(motor_cmd_max == 0 || motor_cmd_max < 0){
        RCLCPP_DEBUG_STREAM(node->get_logger(), "Motor_cmd_max error" << 4);
    }

    declare_parameter("motor_cmd_per_rad_sec", 0.);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();
    if(motor_cmd_per_rad_sec == 0 || motor_cmd_per_rad_sec < 0){
        RCLCPP_DEBUG_STREAM(node->get_logger(), "Motor_cmd_per_rad_sec error" << 4);
    }

    declare_parameter("encoder_ticks_per_rad", 0.);
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    if(encoder_ticks_per_rad == 0 || encoder_ticks_per_rad < 0){
        RCLCPP_DEBUG_STREAM(node->get_logger(), "Encoder_ticks_per_rad error" << 4);
    }

    declare_parameter("collision_radius", 0.);
    collision_radius = get_parameter("collision_radius").as_double();
    if(collision_radius == 0 || collision_radius < 0){
        RCLCPP_DEBUG_STREAM(node->get_logger(), "Collision_radius error" << 4);
    }

    ///Cmd subscription and publisher
    cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&MinimalSubscriber::cmd_vel_callback, this, _1));
    wheel_cmd_publisher_ = this->create_publisher<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10);

    ///subscribe to the sensor data
    sensor_data_subscription_ = this->create_subscription<nuturtlebot_msgs::msg::SensorData>(
        "sensor_data",10, std::bind(&MinimalSubscriber::sensor_callback, this, _1));

    ///provide the angle (in radians) and velocity (in rad/sec) of each wheel
    joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);


    turtlelib::DiffDrive yaml_robot(track_width, wheel_radius);
    robot = yaml_robot;
  }

private:

    void cmd_vel_callback(const geometry::msg::Twist & msg) const
    {   Twist2D twist;
        twist.omega = msg.angular.z;
        twist.x = msg.linear.x;

        ///transfer radians to motor cmd using motor_cmd_per_rad_sec
        double left_velocity = robot.inverseKinematics(twist).l / motor_cmd_per_rad_sec;
        double right_velocity = robot.inverseKinematics(twist).r / motor_cmd_per_rad_sec;
        if(left_velocity > motor_cmd_max){
            left_velocity = motor_cmd_max;
        }
        else if(left_velocity < -motor_cmd_max){
            left_velocity = -motor_cmd_max;
        }
        if(right_velocity > motor_cmd_max){
            right_velocity = motor_cmd_max;
        }
        else if(right_velocity < -motor_cmd_max){
            right_velocity = -motor_cmd_max;
        }
        wheel_cmd_publisher_->publish(WheelCommands{left_velocity, right_velocity});

        RCLCPP_INFO(this->get_logger(), "I heard cmd vel as (x, w): '%d %d'", msg->linear.x, msg->angular.z);
    }

    void sensor_callback(const nuturtlebot_msgs::msg::SensorData & msg) const
    {
        ///NEED IMPLEMENTATION
        RCLCPP_INFO(this->get_logger(), "I heard sensor data as (left, right): '%d %d'", msg.left_encoder, msg.right_encoder);
        double left_encoder = msg.left_encoder;
        double right_encoder = msg.right_encoder;
        std::vector<double> joint_pose_vec(2);
        joint_pose_vec.at(0) = static_cast<double> (left_encoder / encoder_ticks_per_rad);
        joint_pose_vec.at(1) = static_cast<double> (right_encoder / encoder_ticks_per_rad);

        joint_state.position = joint_pose_vec;
        joint_state.header.stamp =  msg.stamp; //why not useing the time rn

        if(initial_js){
            initial_js = false;
        }
        else{
            const auto currTime = joint_state.header.stamp.sec + joint_state/home/jialuyu/ME495_Slam/src/slam-project-NuCapybara/nuturtle_control/src/turtle_control.cpp:143:36: error: assignment of read-only location ‘((const Turtle_control*)this)->Turtle_control::joint_state.sensor_msgs::msg::JointState_<std::allocator<void> >::position.std::vector<double, std::allocator<double> >::at(0)’

            joint_state.velocity = {(joint_state.position[0]- last_jointstate.position[0])/delta_time, (joint_state.position[1] - last_jointstate.position[1])/delta_time};
        }
        joint_state_publisher_->publish(joint_state);
        last_jointstate = joint_state;
    }

    
    
    
    Twist2D twist;
    bool initial_js = true;
    sensor_msgs::msg::JointState joint_state, last_jointstate;
    double wheel_radius, track_width, motor_cmd_max, motor_cmd_per_rad_sec, encoder_ticks_per_rad, collision_radius;
    turtlelib::DiffDrive robot{0.0, 0.0};
    rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
    rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Turtle_control>());
  rclcpp::shutdown();
  return 0;
}
