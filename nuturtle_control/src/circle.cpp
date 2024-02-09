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
#include "geometry_msgs/msg/twist.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nuturtle_control/srv/initial_pose.hpp"
#include "nuturtle_control/srv/control.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using namespace turtlelib;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Circle: public rclcpp::Node
{
public:
  Circle()
  : Node("circle")
  {

    turtlelib::DiffDrive temp(wheel_radius, track_width);
    robot = temp;

    ///parameter delcaration
    this->declare_parameter("rate", 100.);
    rate_hz = get_parameter("rate").as_double();
    std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));
    
    ///angular velocity
    this->declare_parameter("velocity", 0.);
    velocity = get_parameter("velocity").as_double();

    this->declare_parameter("radius", 0.);
    radius = get_parameter("radius").as_double();


    declare_parameter("wheel_radius", -1.);
    wheel_radius = get_parameter("wheel_radius").as_double();
    if(wheel_radius == 0 || wheel_radius < 0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Wheel_radius error" << 4);
    }

    declare_parameter("track_width", -1.);
    track_width = get_parameter("track_width").as_double();
    if(track_width == 0 || track_width < 0){
        RCLCPP_DEBUG_STREAM(get_logger(), "Track_width error" << 4);
    }

    control_ = this->create_service<nuturtle_control::srv::Control>("~/control", std::bind(&Circle::control, this, std::placeholders::_1, std::placeholders::_2));
    stop_ =  this->create_service<std_srvs::srv::Empty>("~/stop", std::bind(&Circle::stop, this, std::placeholders::_1, std::placeholders::_2));
    reverse_ = this->create_service<std_srvs::srv::Empty>("~/reverse", std::bind(&Circle::reverse, this, std::placeholders::_1, std::placeholders::_2));
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = create_wall_timer(
    rate, std::bind(&Circle::timer_callback, this));
    

  }


private:


    void timer_callback(){
      if(ifstop == false){
        /// if stop service is not called, then cmd publisher will continue to publish the command
        cmd_publisher_->publish(command);
        RCLCPP_INFO_STREAM_ONCE(get_logger(), "Publish a command thru timecallback_circle!");
      }
    }

    void control(std::shared_ptr<nuturtle_control::srv::Control::Request> request, std::shared_ptr<nuturtle_control::srv::Control::Response> response){
        ifstop = false;
        velocity = request->angular_velocity;
        radius = request->radius;
        response->success = true;
        command.linear.x = velocity*radius;
        command.angular.z = velocity;
    }

    void reverse(std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response){
        if(velocity !=0 && radius != 0){
          ifstop = false;
          command.linear.x = velocity*radius; /// the linear velocity is changed to the opposite direction based on angular velocity
          command.angular.z = -velocity; /// the angular velocity is changed to the opposite direction
        }
    }

    
    void stop(std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response){
        ifstop = true; ///QUESTIONS
        command.linear.x = 0.0;
        command.angular.z = 0.0;
        cmd_publisher_->publish(command);

    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    bool ifstop = false;
    double rate_hz;
    geometry_msgs::msg::Twist command;
    double radius, velocity;
    double track_width, wheel_radius;
    turtlelib::DiffDrive robot= turtlelib::DiffDrive(0.0, 0.0);
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
