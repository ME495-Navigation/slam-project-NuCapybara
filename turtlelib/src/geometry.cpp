#include "turtlelib/geometry2d.hpp"
#include <iostream>

namespace turtlelib{
    std::ostream & operator<<(std::ostream & os, const Point2D & p){
        os << "[" << p.x << " " << p.y << "]";
        return os;
    }


    /// \brief input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to readgeome
    /// HINT: See operator>> for Vector2D
    std::istream & operator>>(std::istream & is, Point2D & p){
        char ch;
        char trash;
        is >> ch; //omit the "["
        if(ch != '['){
            p.x = ch;
            is >> trash;
            is >> p.y;
        }
        // if the first one starts with [
        else{
            //the next one for input is p.x
            is >> p.x;
            is >> trash;
            is >> p.y;
        }
        
        return is;
    }
    /// \brief wrap an angle to (-PI, PI]
    /// \param rad (angle in radians)
    /// \return an angle equivalent to rad but in the range (-PI, PI]
    double normalize_angle(double rad){
        while(rad < -PI){
            rad += 2*PI;
        }
        while (rad > PI){
            rad -= 2*PI;
        }
        return rad;
    }


    /// \brief Subtracting one point from another yields a vector
    /// \param head point corresponding to the head of the vector
    /// \param tail point corresponding to the tail of the vector
    /// \return a vector that points from p1 to p2
    /// NOTE: this is not implemented in terms of -= because subtracting two Point2D yields a Vector2D
    Vector2D operator-(const Point2D & head, const Point2D & tail){
        Vector2D v;
        v.x = head.x -tail.x;
        v.y = head.y - tail.y;
        return v;
    }

    /// \brief Adding a vector to a point yields a new point displaced by the vector
    /// \param tail The origin of the vector's tail
    /// \param disp The displacement vector
    /// \return the point reached by displacing by disp from tail
    /// NOTE: this is not implemented in terms of += because of the different types
    Point2D operator+(const Point2D & tail, const Vector2D & disp){
        Point2D pt;
        pt.x = tail.x + disp.x;
        pt.y = tail.y + disp.y;
        return pt;
    }

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// We have lower level control however. For example:
    /// peek looks at the next unprocessed character in the buffer without removing it
    /// get removes the next unprocessed character from the buffer.
    std::istream & operator>>(std::istream & is, Vector2D & v){
        is >> std::ws;
        char input;
        input = is.peek();
        //Sceanario: the first char is [
        if(input == '['){
            is.get(); //consume [
            double x;
            double y;
            is >> x;
            is.get(); //consumes the whitespace between x and y
            is >> y;
            if(is.get()==']'){
                v.x = x;
                v.y = y;
            }
            else{
                //if the last one is not ], then fails
                 is.setstate(std::ios::failbit);
            }
        }
        else{
            double x;
            double y;
            is >> x;
            is >> y;
            v.x = x;
            v.y = y;
        }
        return is;
    }
    
}; 