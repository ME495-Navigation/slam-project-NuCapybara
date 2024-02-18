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
///     ~/red/path (nav_msgs::msg::Path): path of the turtlebot
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
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/geometry2d.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("nusim"), timestep_(0)
  {
    ///create a timer
    declare_parameter("rate", 200.);
    rate_hz = get_parameter("rate").as_double();
    std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));

    declare_parameter("x", 0.);
    x_i = get_parameter("x").as_double();

    declare_parameter("y", 0.);
    y_i = get_parameter("y").as_double();

    declare_parameter("theta", 0.);
    theta_i = get_parameter("theta").as_double();

    declare_parameter("arena_x_length", 0.);
    arena_x_length = get_parameter("arena_x_length").as_double();

    declare_parameter("arena_y_length", 0.);
    arena_y_length = get_parameter("arena_y_length").as_double();

    declare_parameter("obstacles/x", std::vector<double>{});
    obx = get_parameter("obstacles/x").as_double_array();

    declare_parameter("obstacles/y", std::vector<double>{});
    oby = get_parameter("obstacles/y").as_double_array();

    declare_parameter("obstacles/r", 0.0);
    obr = get_parameter("obstacles/r").as_double();

    declare_parameter("motor_cmd_per_rad_sec", 0.0);
    motor_cmd_per_rad_sec = get_parameter("motor_cmd_per_rad_sec").as_double();

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

    declare_parameter("encoder_ticks_per_rad", -1.);
    encoder_ticks_per_rad = get_parameter("encoder_ticks_per_rad").as_double();
    if(encoder_ticks_per_rad == 0 || encoder_ticks_per_rad < 0){
        RCLCPP_DEBUG_STREAM(get_logger(), "encoder_ticks_per_rad error" << 4);
    }

    
    initial = true;
    x0 = 0.0;
    y0 = 0.0;
    theta0 = 0.0;
    timer_ = create_wall_timer(
      rate, std::bind(&Nusim::timer_callback, this));
    //timestep publisher
    time_step_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    //service reset
    reset_ =
      create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));

    //broadcaster the transform
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //teleport service
    teleport_ =
      create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport, this, std::placeholders::_1, std::placeholders::_2));

    wheel_subscriber_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "red/wheel_cmd", 10, std::bind(&Nusim::wheel_callback, this, std::placeholders::_1));
// sensor data publisher
    sensor_data_pub = this->create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "red/sensor_data",
      10);
      
    path_pub = create_publisher<nav_msgs::msg::Path>("red/path", 10);
    
    
    //wall publisher
    rclcpp::QoS qos_policy = rclcpp::QoS(rclcpp::KeepLast(10)).transient_local();
    wall_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/walls", qos_policy);
    obs_pub = create_publisher<visualization_msgs::msg::MarkerArray>("~/obstacles", qos_policy);
    
  }

private:
  void timer_callback()
  {
    //if its the first time calling initial, set the initial location
    if (initial) {
      x0 = x_i;
      y0 = y_i;
      theta0 = theta_i;
      initial = false;
    }
    // publish timetsep
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_;
    time_step_publisher_->publish(message);
    timestep_++;
    

    wallPub();
    obstacle_publisher();

    ///update the robot configuration
    auto delta_left_angle = left_velocity/rate_hz;///velocity(rad/s)*time to get rad
    auto delta_right_angle = right_velocity/rate_hz;
    auto new_right_angle = delta_right_angle + robot.get_wheel_state().r;
    auto new_left_angle = delta_left_angle + robot.get_wheel_state().l;
    ///use delta wheel degree to update wheel state
    RCLCPP_INFO_STREAM(get_logger(), "left=" << delta_left_angle << " right=" << delta_right_angle);
    robot.forwardKinematics(turtlelib::WheelState{delta_left_angle, delta_right_angle});

    x = robot.get_current_config().translation().x;
    y = robot.get_current_config().translation().y;
    theta = robot.get_current_config().rotation();

    send_transform(x, y, theta);
    //update the sensor call back
    sensor_data.left_encoder = static_cast<int>(new_left_angle*encoder_ticks_per_rad);
    sensor_data.right_encoder = static_cast<int>(new_right_angle*encoder_ticks_per_rad);
    sensor_data.stamp = get_clock()->now();
    sensor_data_pub->publish(sensor_data);

    //publish the nav_msgs::msg::Path
    red_path.header.frame_id = "nusim/world";
    red_path.header.stamp = get_clock()->now();
    path_pub->publish(red_path);
  }


  void reset(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RESET IS GOING ON!");
    timestep_ = 0;
    // RCLCPP_INFO_STREAM(get_logger(), "reset x=" << x << " y=" << y << " theta=" << theta);
    x = x_i;
    y = y_i;
    theta = theta_i;
  }

  //broadcast the location by giving x y w
  //also update the robot path with x y w
  void send_transform(double x, double y, double w)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, w);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    //update the robot path
    robot_pose.header.frame_id = "nusim/world";
    robot_pose.header.stamp = get_clock()->now();
    robot_pose.pose.position.x = x;
    robot_pose.pose.position.y = y;
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation.x = q.x();
    robot_pose.pose.orientation.y = q.y();
    robot_pose.pose.orientation.z = q.z();
    robot_pose.pose.orientation.w = q.w();
    // RCLCPP_INFO_STREAM(
      // get_logger(), "Send path pose as x=" << x << " y=" << y << " theta=" << w);
    red_path.poses.push_back(robot_pose);

  }

  void teleport(
    std::shared_ptr<nusim::srv::Teleport::Request> req,
    std::shared_ptr<nusim::srv::Teleport::Response> res)
  {
    x = req->x;
    y = req->y;
    theta = req->theta;
    RCLCPP_INFO_STREAM(
      get_logger(), "Teleporting service as x=" << x << " y=" << y << " theta=" << theta0);
    res->success = true;
  }

  void wallPub()
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;

    for (int i = 0; i < 4; i++) {
      m.header.stamp = this->get_clock()->now();
      m.header.frame_id = "nusim/world";
      m.id = i;
      m.type = 1;
      m.action = 0;

      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      //wall scale
      m.scale.x = 0.0;
      m.scale.y = 0.0;
      m.scale.z = 0.25;
      m.pose.position.x = 0.0;
      m.pose.position.y = 0.0;
      m.pose.position.z = 0.125;
      arr.markers.push_back(m);
    }
    const auto wall_thickness = 0.1;

    arr.markers.at(0).scale.x = wall_thickness;
    arr.markers.at(0).scale.y = arena_y_length + 2 * wall_thickness;
    arr.markers.at(0).pose.position.x = 0.5 * (arena_x_length + wall_thickness);

    arr.markers.at(1).scale.x = arena_x_length + 2 * wall_thickness;
    arr.markers.at(1).scale.y = wall_thickness;
    arr.markers.at(1).pose.position.y = 0.5 * (arena_y_length + wall_thickness);

    arr.markers.at(2).scale.x = wall_thickness;
    arr.markers.at(2).scale.y = arena_y_length + 2 * wall_thickness;
    arr.markers.at(2).pose.position.x = -0.5 * (arena_x_length + wall_thickness);

    arr.markers.at(3).scale.x = arena_x_length + 2 * wall_thickness;
    arr.markers.at(3).scale.y = wall_thickness;
    arr.markers.at(3).pose.position.y = -0.5 * (arena_y_length + wall_thickness);

    wall_pub->publish(arr);
  }

  void obstacle_publisher()
  {
    if (oby.size() == obx.size()) {
      RCLCPP_INFO_ONCE(get_logger(), "Valid Marker Input!");
      cyl_num = oby.size();
    } else {
      RCLCPP_DEBUG(get_logger(), "Invalid Marker Input!");
      cyl_num = 0;
      return;
    }
    visualization_msgs::msg::MarkerArray arr_obstacle;
    for (size_t i = 0; i < cyl_num; i++) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = this->get_clock()->now();
      m.header.frame_id = "nusim/world";
      m.id = i;
      m.type = 3;
      m.action = 0;

      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      //wall scale
      m.scale.x = this->obr * 2;
      m.scale.y = this->obr * 2;
      m.scale.z = 0.25;
      m.pose.position.x = this->obx[i];
      m.pose.position.y = this->oby[i];
      m.pose.position.z = 0.125;
      arr_obstacle.markers.push_back(m);
    }
    obs_pub->publish(arr_obstacle);
  }

  void wheel_callback(const nuturtlebot_msgs::msg::WheelCommands & wc)
  {
    // just store left and right velocity and do this update in the timer
    left_velocity = wc.left_velocity * motor_cmd_per_rad_sec;   //wc in ticks, get the velocity in rad/s
    right_velocity = wc.right_velocity * motor_cmd_per_rad_sec;
    // RCLCPP_INFO_STREAM(
    //   get_logger(), "Teleporting service as x=" << left_velocity << " y=" << right_velocity);
  }

  double x_i, y_i, theta_i;
  double x, y, theta, rate_hz, encoder_ticks_per_rad;
  double arena_x_length, arena_y_length;
  double track_width, wheel_radius, motor_cmd_per_rad_sec;
  std::vector<double> obx, oby;
  double obr;
  bool initial;
  double x0, y0, theta0;
  size_t timestep_;
  size_t cyl_num;
  double left_velocity=0.0; //rad/s
  double right_velocity=0.0;
  nav_msgs::msg::Path red_path; //the path of the robot
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_step_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_subscriber_;
  turtlelib::DiffDrive robot{0.033, 0.16};
  nuturtlebot_msgs::msg::SensorData sensor_data;
  geometry_msgs::msg::PoseStamped robot_pose = geometry_msgs::msg::PoseStamped();

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
