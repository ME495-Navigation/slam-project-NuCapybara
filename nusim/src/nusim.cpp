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
    x = get_parameter("x").as_double();

    declare_parameter("y", 0.);
    y = get_parameter("y").as_double();

    declare_parameter("theta", 0.);
    theta = get_parameter("theta").as_double();

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

    turtlelib::DiffDrive temp(wheel_radius, track_width);
    robot = temp;
    
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
      x0 = x;
      y0 = y;
      theta0 = theta;
      initial = false;
    }
    // publish timetsep
    auto message = std_msgs::msg::UInt64();
    message.data = timestep_;
    time_step_publisher_->publish(message);
    timestep_++;
    send_transform(x, y, theta);

    wallPub();
    obstacle_publisher();

    ///update the robot configuration
    auto delta_left_angle = left_velocity/rate_hz;///velocity(rad/s)*time to get rad
    auto delta_right_angle = right_velocity/rate_hz;
    auto new_right_angle = delta_right_angle + robot.get_wheel_state().r;
    auto new_left_angle = delta_left_angle + robot.get_wheel_state().l;
    ///use delta wheel degree to update wheel state
    auto new_wheel_state_delta = turtlelib::WheelState{delta_left_angle, delta_right_angle};
    robot.forwardKinematics(new_wheel_state_delta);
    
    //update the sensor call back
    nuturtlebot_msgs::msg::SensorData sensor_data;
    sensor_data.left_encoder = static_cast<int>(new_right_angle*motor_cmd_per_rad_sec);
    sensor_data.right_encoder = static_cast<int>(new_left_angle*motor_cmd_per_rad_sec);
    sensor_data.stamp = get_clock()->now();
    sensor_data_pub->publish(sensor_data);

  }


  void reset(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RESET IS GOING ON!");
    timestep_ = 0;
    RCLCPP_INFO_STREAM(get_logger(), "reset x=" << x0 << " y=" << y0 << " theta=" << theta0);
    x = x0;
    y = y0;
    theta = theta0;
  }

  //broadcast the location by giving x y w
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

  }


  double x, y, theta, rate_hz;
  double arena_x_length, arena_y_length;
  double track_width, wheel_radius, motor_cmd_per_rad_sec;
  std::vector<double> obx, oby;
  double obr;
  bool initial;
  double x0, y0, theta0;
  size_t timestep_;
  size_t cyl_num;
  int left_velocity = 0; //rad/s
  int right_velocity = 0;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_step_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_pub;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_pub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_subscriber_;
  turtlelib::DiffDrive robot = turtlelib::DiffDrive(0.0, 0.0);
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nusim>());
  rclcpp::shutdown();
  return 0;
}
