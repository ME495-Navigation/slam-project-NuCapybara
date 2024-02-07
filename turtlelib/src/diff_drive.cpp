#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <stdexcept>

namespace turtlelib
{   

    DiffDrive::DiffDrive(double wheel_radius, double track_width):
        w{WheelState{0.0, 0.0}},
        q{Transform2D()},
        wheel_radius{wheel_radius},
        track_width{track_width}
    {}

    DiffDrive::DiffDrive(Transform2D config, double wheel_radius, double track_width):
        w{WheelState{0.0, 0.0}},
        q{config},
        wheel_radius{wheel_radius},
        track_width{track_width}
    {}


    double DiffDrive::get_radius(){
        return wheel_radius;
    }

    double DiffDrive::get_track_width(){
        return track_width;
    }

    WheelState DiffDrive::get_wheel_state(){
        return w;
    }
    
    Transform2D DiffDrive:: get_current_config(){
        return q;
    }
    void DiffDrive::forwardKinematics(WheelState newWheelState){
        //eq 13.15
        double newLeft = newWheelState.l;
        double newRight = newWheelState.r;
        auto dPhi = -wheel_radius/(track_width)*newLeft + wheel_radius/(track_width)*newRight;
        auto dx = wheel_radius/2*cos(q.rotation())*newLeft + wheel_radius/2*cos(q.rotation())*newRight;
        auto dy = wheel_radius/2*sin(q.rotation())*newLeft + wheel_radius/2*sin(q.rotation())*newRight;

        w.l += newLeft;
        w.r += newRight;
        auto newPhi = normalize_angle(q.rotation() + dPhi);
        auto newdx = q.translation().x + dx;
        auto newdy = q.translation().y + dy;
        q = Transform2D(Vector2D{newdx, newdy}, newPhi);
    }

    WheelState DiffDrive::inverseKinematics(Twist2D twist){
        if(almost_equal(twist.y, 0)){
            const auto omega = twist.omega;
            const auto tx = twist.x;
            ///MR Formula 13.34
            auto newleftWheel = 1/(2*wheel_radius)*(2*tx - track_width*omega);
            auto newrightWheel = 1/(2*wheel_radius)*(track_width*omega + 2*tx);
            return WheelState{newleftWheel, newrightWheel};
        }
        else{
            throw std::logic_error("Slip existing, twist y component not equal to 0.");
        }
            

    }
}
