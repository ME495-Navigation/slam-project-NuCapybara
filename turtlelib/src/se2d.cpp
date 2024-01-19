#include<iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include"turtlelib/se2d.hpp"

namespace turtlelib
{

    {
        /// \brief the angular velocity
        double omega = 0.0;

        /// \brief the linear x velocity
        double x = 0.0;

        /// \brief the linear y velocity
        double y = 0.0;
    };

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

        Vector2D operator()(Vector2D v) const{
            const auto new_x = v.x*cos(angular)-sin(angular)*v.y; //for vector there is no translation
            const auto new_y = v.x*sin(angular)+cos(angular)*v.y;
            return Vector2D{new_x, new_y};
        }

        /// \brief apply a transformation to a Twist2D (e.g. using the adjoint)
        /// \param v - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D v) const{
            Twist2D newTwist;
            newTwist.x = v.x;
            newTwist.y = v.x*linear.y + v.y*cos(angular)-v.omega*sin(angular);
            newTwist.omega = -v.x*linear.x + v.y*sin(angular) + v.omega*cos(angular);
            return newTwist;
        }

        /// \brief invert the transformation
        /// \return the inverse transformation.
        Transform2D inv() const{
            Transform2D newTrans;
            newTrans.linear.x = -x*cos(angular)-y*sin(angular);
            newTrans.linear.y = -y*cos(angular)+x*sin(angular);
            auto oldAngular = angular;
            newTrans.angular = -oldAngular;
            return newTrans;
        }

        /// \brief compose this transform with another and store the result
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & Transform2D::operator*=(const Transform2D & rhs){
            const auto new_angular = angular + rhs.angular;
            const auto new_x = rhs.linear.x*cos(angular) - rhs.linear.y*sin(angular) + linear.x;
            const auto new_y = rhs.linear.x*sin(angular) + rhs.linear.y*cos(angular) + linear.y;

            angular = new_angular;
            linear.x = new_x;
            linear.y = new_y;
            return *this;
        }


        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const{
            return linear;
        }


        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const{
            return angular;
        }

    };


    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << tf.angular <<  " x: " << tf.linear.x << " y: " << tf.linear.y; 
    }

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
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

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        return lhs*=rhs;
    }



#endif
