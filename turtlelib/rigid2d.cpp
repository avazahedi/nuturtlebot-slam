/// \file
/// \brief Two-dimensional rigid body transformations.

#include <iostream>
#include "rigid2d.hpp"

namespace turtlelib {

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// \param is - stream from which to read
    /// \param v [out] - output vector
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user types
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin) and processing stops.
    ///
    /// We have lower level control however.
    /// std::peek() looks at the next unprocessed character in the buffer without removing it
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// std::get() removes the next unprocessed character from the buffer.
    ///     https://en.cppreference.com/w/cpp/io/basic_istream/get
    /// When you call std::peek() it will wait for there to be at least one character in the buffer (e.g., the user types a character)
    /// HINT: this function can be written in under 20 lines and uses only std::peek(), std::get(), istream::operator>>() and a little logic
    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        char c = is.peek();
        if (c == '[') {
            is.get();
        }
        is >> v.x;
        is.get();
        is >> v.y;
        is.get();

        return is;
    }




    /// \brief Create an identity transformation
    turtlelib::Transform2D::Transform2D()
    {
        
    }

    // /// \brief create a transformation that is a pure translation
    // /// \param trans - the vector by which to translate
    // explicit Transform2D(turtlelib::Vector2D trans)
    // {

    // }

    // /// \brief create a pure rotation
    // /// \param radians - angle of the rotation, in radians
    // explicit Transform2D(double radians)
    // {

    // }

    // /// \brief Create a transformation with a translational and rotational
    // /// component
    // /// \param trans - the translation
    // /// \param rot - the rotation, in radians
    // Transform2D(Vector2D trans, double radians)
    // {
        
    // }

    // /// \brief apply a transformation to a Vector2D
    // /// \param v - the vector to transform
    // /// \return a vector in the new coordinate system
    // Vector2D operator()(Vector2D v) const
    // {
        
    // }


    // /// \brief invert the transformation
    // /// \return the inverse transformation. 
    // Transform2D inv() const
    // {

    // }

    // /// \brief compose this transform with another and store the result 
    // /// in this object
    // /// \param rhs - the first transform to apply
    // /// \return a reference to the newly transformed operator
    // Transform2D & operator*=(const Transform2D & rhs);

    // /// \brief the translational component of the transform
    // /// \return the x,y translation
    // Vector2D translation() const;

    // /// \brief get the angular displacement of the transform
    // /// \return the angular displacement, in radians
    // double rotation() const;

    // /// \brief \see operator<<(...) (declared outside this class)
    // /// for a description
    // friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);

}


int main(void) {
    std::cout << turtlelib::deg2rad(180) << "\n";

    struct turtlelib::Vector2D v;
    v.x = 1.4;
    v.y = 2.8;

    std::cout << v << std::endl;

    struct turtlelib::Vector2D w;
    std::cin >> w;

    std::cout << w << std::endl;

    // std::cout << "Hello";
    return 0;
}