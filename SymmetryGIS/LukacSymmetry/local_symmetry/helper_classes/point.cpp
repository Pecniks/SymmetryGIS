#include <pch.h>
#include <cmath>
#include <sstream>

#include "point.h"
#include "tolerance.h"

using namespace Symmetry;



// CONSTRUCTORS
// Default constructor without parameters.
template <class T>
Point<T>::Point() :
    x(static_cast<T>(0)),
    y(static_cast<T>(0)),
    z(static_cast<T>(0))
{}

// Constructor with X, Y and Z coordinates.
template <class T>
Point<T>::Point(T x, T y, T z) :
    x(x),
    y(y),
    z(z)
{}

// Copy constructor.
template <class T>
Point<T>::Point(const Point& p) :
    x(p.getX()),
    y(p.getY()),
    z(p.getZ())
{}




// GETTERS AND SETTERS
// X coordinate getter.
template <class T>
const T Point<T>::getX() const {
    return x;
}

// X coordinate setter.
template <class T>
void Point<T>::setX(const T x) {
    this->x = x;
}

// Y coordinate getter.
template <class T>
const T Point<T>::getY() const {
    return y;
}

// Y coordinate setter.
template <class T>
void Point<T>::setY(const T y) {
    this->y = y;
}

// Z coordinate getter.
template <class T>
const T Point<T>::getZ() const {
    return z;
}

// Z coordinate setter.
template <class T>
void Point<T>::setZ(const T z) {
    this->z = z;
}



// CLASS METHODS
// Calculation of the Euclidean distance between two points.
template <class T>
double Point<T>::distance(Point<T> v1, Point<T> v2) {
    return (
        sqrt(pow(v1.x - v2.x, 2) +
             pow(v1.y - v2.y, 2) +
             pow(v1.z - v2.z, 2)
        )
    );
}

// Calculation of the midpoint of a line segment.
template <class T>
Point<T> Point<T>::calculateMidpoint(Point<T> p1, Point<T> p2) {
    // Midpoint can be calculated as the "average point" between two points: (p1 + p2) / 2.
    //Point<T> midpoint((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0, (p1.z + p2.z) / 2.0);
    Point<T> midpoint = (p1 + p2) / 2;
    return midpoint;
}

// Calculation of a barycenter point (centroid).
template <class T>
Point<double> Point<T>::calculateBarycenterPoint(std::vector<Point<double>>& points) {
    Point<double> p;
    for (Point<double>& point : points) {
        p += point;
    }
    p /= (double)points.size();

    return p;
}

// Cross product of two vectors.
template <class T>
Point<double> Point<T>::crossProduct(const Point<double>& lhs, const Point<double>& rhs) {
    return {
        lhs.getY() * rhs.getZ() - lhs.getZ() * rhs.getY(),
        lhs.getZ() * rhs.getX() - lhs.getX() * rhs.getZ(),
        lhs.getX() * rhs.getY() - lhs.getY() * rhs.getX()
    };
}




// OBJECT METHODS
// Converting a point to a string.
template <class T>
std::string Point<T>::toString() const {
    std::stringstream ss;
    ss << "Point(" << x << "," << y << "," << z << ")";
    return ss.str();
}

// Sum of point coordinates and a number.
template <class T>
Point<T> Point<T>::operator + (const T &number) const {
    return Point<T>(x + number, y + number, z + number);
}

// Adding a vector to the point.
template <class T>
Point<T> Point<T>::operator + (Point<T>& p) const {
    return Point<T>(x + p.x, y + p.y, z + p.z);
}

// Sum of two points.
template <class T>
Point<T> Point<T>::operator += (Point<T>& p) {
    x += p.x;
    y += p.y;
    z += p.z;

    return *this;
}

// Difference of point coordinates and a number.
template <class T>
Point<T> Point<T>::operator - (const T &number) const {
    return Point<T>(x - number, y - number, z - number);
}

// Difference between two points is calculated
// as the distance between the pairs of coordinates.
template <class T>
Point<T> Point<T>::operator - (const Point<T>& p) const {
    return Point<T>(x - p.x, y - p.y, z - p.z);
}

// Multiplying of a point and a scalar.
template <class T>
Point<T> Point<T>::operator * (const T &factor) const {
    return Point<T>(factor * x, factor * y, factor * z);
}

// Multiplying of a vector and a point.
template <class T>
Point<T> Point<T>::operator * (Point<T>& p) const {
    return Point<T>(p.x * x, p.y * y, p.z * z);
}

// Division of point coordinates and a number.
template <class T>
Point<T> Point<T>::operator / (const T &factor) const {
    return Point<T>(x / factor, y / factor, z / factor);
}

// Division of point coordinates and a number.
template <class T>
Point<T> Point<T>::operator /= (const T &factor) {
    x /= factor;
    y /= factor;
    z /= factor;

    return *this;
}

// Checking whether the two points are the same.
template <class T>
bool Point<T>::operator == (Point<T> p) const {
    return Tolerance::isInTolerance(x, p.getX(), 0.01) && Tolerance::isInTolerance(y, p.getY(), 0.01) && Tolerance::isInTolerance(z, p.getZ(), 0.01);
}

// Checking whether the two points are not the same.
template <class T>
bool Point<T>::operator != (Point p) const {
    return !Tolerance::isInTolerance(x, p.getX(), 0.01) || !Tolerance::isInTolerance(y, p.getY(), 0.01) || !Tolerance::isInTolerance(z, p.getZ(), 0.01);
}

// Dot product of two vectors.
template <class T>
T Point<T>::dot(const Point<T>& p1) const {
    const T xDot = p1.x * x;
    const T yDot = p1.y * y;
    const T zDot = p1.z * z;

    return xDot + yDot + zDot;
}

// Getting length of a vector (point).
template <class T>
double Point<T>::length() const {
    const T xDot = x * x;
    const T yDot = y * y;
    const T zDot = z * z;
    const T dot = xDot + yDot + zDot;

    return std::sqrt(dot);
}

// Vector normalization.
template <class T>
Point<T> Point<T>::normalize() {
    auto len = this->length();
    return Point<T>(static_cast<T>(x / len), static_cast<T>(y / len), static_cast<T>(z / len));
}

//template<typename T>
//template<typename T2>
//Point<T> Point<T>::convert(const Point<T2> &point)
//{
//    return Point<T>(static_cast<T>(point.getX()), static_cast<T>(point.getY()), static_cast<T>(point.getZ()));
//}

namespace Symmetry {
    template class Point<int>;
    template class Point<float>;
    template class Point<double>;
};
