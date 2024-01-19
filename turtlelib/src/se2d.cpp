#include<iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include"turtlelib/se2d.hpp"

namespace turtlelib
{

    std::ostream & operator<<(std::ostream & os, const Twist2D & tw){
        os << "[" << tw.omega << " " << tw.x << " " <<  tw.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw){
        char ch;
        ch = is.peek();
        if(ch != '['){
            is >> tw.omega;
            is >> tw.x;
            is >> tw.y;
        }
        // if the first one starts with [
        else{
            //consume "["
            is.get();
            is >> tw.omega;
            is >> tw.x;
            is >> tw.y;
        }
        return is;
    }




        Transform2D::Transform2D():
            linear{Vector2D{0,0}},
            angular{0}
            {}


        Transform2D::Transform2D{Vector2D trans}:
            linear{trans},
            angular{0}
            {}


        Transform2D::Transform2D(double radians):
            linear{Vector2D{0,0}},
            angular{radians}
            {}

        Transform2D::Transform2D(Vector2D trans, double radians):
            linear{trans},
            angular{radians}
            {}

        Transform2D::Point2D operator()(Point2D p) const{
            Point2D newPt;
            newPt.x = p.x*cos(angular)-sin(angular)*p.y+linear.x;
            newPt.y = p.x*sin(angular)+cos(angular)*p.y+linear.y;
            return newPt;
        }

        Transform2D::Vector2D operator()(Vector2D v) const{
            const auto new_x = v.x*cos(angular)-sin(angular)*v.y; //for vector there is no translation
            const auto new_y = v.x*sin(angular)+cos(angular)*v.y;
            return Vector2D{new_x, new_y};
        }

        Transform2D::Twist2D operator()(Twist2D v) const{
            Twist2D newTwist;
            newTwist.x = v.x;
            newTwist.y = v.x*linear.y + v.y*cos(angular)-v.omega*sin(angular);
            newTwist.omega = -v.x*linear.x + v.y*sin(angular) + v.omega*cos(angular);
            return newTwist;
        }

        Transform2D::Transform2D inv() const{
            Transform2D newTrans;
            newTrans.linear.x = -x*cos(angular)-y*sin(angular);
            newTrans.linear.y = -y*cos(angular)+x*sin(angular);
            auto oldAngular = angular;
            newTrans.angular = -oldAngular;
            return newTrans;
        }

        Transform2D::Transform2D & Transform2D::operator*=(const Transform2D & rhs){
            const auto new_angular = angular + rhs.angular;
            const auto new_x = rhs.linear.x*cos(angular) - rhs.linear.y*sin(angular) + linear.x;
            const auto new_y = rhs.linear.x*sin(angular) + rhs.linear.y*cos(angular) + linear.y;

            angular = new_angular;
            linear.x = new_x;
            linear.y = new_y;
            return *this;
        }

        Transform2D::Vector2D translation() const{
            return linear;
        }


        Transform2D::double rotation() const{
            return angular;
        }




    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << tf.angular <<  " x: " << tf.linear.x << " y: " << tf.linear.y; 
    }


    std::istream & operator>>(std::istream & is, Transform2D & tf){
        // Remove whitespace
        is >> std::ws;
        double ang_val, x_val, y_val;
        std::string label_ang, label_x, label_y;
        if(is.peek()=="d"){
            is >> label_ang >> ang_val >> label_x >> x_val >> label_y >> y_val;
        }else{
            is >> input_ang >> input_x >> input_y;
        }
        // Read \n character?????????????????
        c = is.get(); 
        
        tf = Transform2D(Vector2D{input_x, input_y},deg2rad(input_ang));
        return is;
    }


    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        return lhs*=rhs;
    }

};

#endi
