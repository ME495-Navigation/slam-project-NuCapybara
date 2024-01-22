#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/TransformStamped"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Nusim : public rclcpp::Node
{
  public:
    Nusim()
    : Node("nusim"), timestep_(0)
    {
    ///create a timer 
    this->declare_parameter("rate", 200.);

    declare_parameter("x0", 0.);
    x0 = get_parameter("x0").as_double();create_publisher

    declare_parameter("y0", 0.);
    y0 = get_parameter("y0").as_double();

    declare_parameter("theta0", 0.);
    theta0 = get_parameter("theta0").as_double();

    declare_parameter("arena_x_length", 0.);
    arena_x_length = get_parameter("arena_x_length").as_double();

    declare_parameter("arena_y_length", 0.);
    arena_y_length = get_parameter("arena_y_length").as_double();
    double arena_height = 0.25;

    bool initial = true;
    double init_x = 0.0;
    double init_y = 0.0;
    double init_theta = 0.0;
    timer_ = this->create_wall_timer(
      rate, std::bind(&Nusim::timer_callback, this));
    //timestep publisher
    time_step_publisher_ = create_publisher<std_msgs::msg::UInt64>("~/timestep", 10);
    //service reset
    reset_ = create_service<std_srvs::srv::Empty>("~/reset", std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));

    //broadcaster the transform
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //teleport service
    teleport = create_service<nusim::srv::Teleport>("~/teleport", std::bind(&Nusim::teleport, this, std::placeholders::_1, std::placeholders::_2));
    
    //wall publisher
    wall_pub = create_publisher<visualization_msgs::msg::Marker_array>("~/walls", 10);
    }



  private:
    void timer_callback()
    {
     //if its the first time calling initial, set the initial location
      if(initial){
        double init_x = x0;
        double init_y = y0;
        double init_theta = theta0;
        initial = false;
      } 
      // publish timetsep  
      auto message = std_msgs::msg::UInt64();
      message.data = timestep_;
      time_step_publisher_->publish(message);
      timestep_++;
      send_transform(x0, y0, theta0);
    }



    void reset(std::shared_ptr<std_srvs::srv::Empty::Request>,
     std::shared_ptr<std_srvs::srv::Empty::Response>){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "RESET IS GOING ON!");
        timestep_ = 0;
        send_transform(init_x, init_y, init_theta);
    }

    //broadcast the location by giving x y w
    void send_transform(double x, double y, double w){
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
        std::shared_ptr<nusim::srv::Teleport::Response> res){
        x0 = req->x;
        y0 = req->y;
        theta0 = req->theta;
        RCLCPP_INFO_STREAM(get_logger(), "Teleporting service as x=" << x0 << " y=" << y0 << " theta=" << theta0);
        res->success = true;
        }

    void wallPub(){
        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker m1, m2, m3, m4;
        MarkerArray.markers.push_back(m1);
        MarkerArray.markers.push_back(m2);
        MarkerArray.markers.push_back(m3);
        MarkerArray.markers.push_back(m4);

        for(int i = 0; i < 4; i++){
            arr.markers.at(i).header.stamp = this->get_clock()->now();
            arr.markers.at(i).header.frame_id = "nusim/world";
            arr.markers.at(i).id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;

            arr.markers.at(i).color.r = 1.0;
            arr.markers.at(i).color.g = 0.0;
            arr.markers.at(i).color.b = 0.0;
            arr.markers.at(i).color.a = 1.0;
            //wall scale
            arr.markers.at(i).scale.x = 0.0;
            arr.markers.at(i).scale.y = 0.0;
            arr.markers.at(i).scale.z = 0.25;
            arr.markers.at(i).pose.position.x = 0.0;
            arr.markers.at(i).pose.position.y = 0.0;
            arr.markers.at(i).pose.position.z = 0.125;
        }
        const auto wall_thickness = 0.1;

        arr.markers.at(0).scale.x = wall_thickness;
        arr.markers.at(0).scale.y = arena_y_length + 2 * wall_thickness;
        arr.markers.at(0).pose.position.x = 0.5 * (arena_x_length + wall_thickness);

        arr.markers.at(1).scale.x = arena_x_length + 2 * wall_thickness;
        arr.markers.at(1).scale.y = wall_thickness;
        arr.markers.at(1).pose.position.y = 0.5 * (arena_y_length + wall_thickness);

        arr.markers.at(2).scale.x = wall_thickness;
        arr.markers.at(2).scale.y = y_length + 2 * wall_thickness;
        arr.markers.at(2).pose.position.x = -0.5 * (arena_x_length + wall_thickness);

        arr.markers.at(3).scale.x = arena_x_length + 2 * wall_thickness;
        arr.markers.at(3).scale.y = wall_thickness;
        arr.markers.at(3).pose.position.y = -0.5 * (arena_y_length + wall_thickness);

        walls_pub_->publish(arr);
    }

        




    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr time_step_publisher_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_;
    size_t timestep_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<>(Nusim));
  rclcpp::shutdown();
  return 0;
}