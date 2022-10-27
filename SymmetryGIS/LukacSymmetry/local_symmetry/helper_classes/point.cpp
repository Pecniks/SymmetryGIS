#include <pch.h>
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

// Copy constructor from vec3.
template <class T>
Point<T>::Point(const LAS::Data::Vector3d& v) :
    x((T)v.x),
    y((T)v.y),
    z((T)v.z)
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
    Point<T> midpoint((p1.x + p2.x) / (T)2.0, (p1.y + p2.y) / (T)2.0, (p1.z + p2.z) / (T)2.0);
    return midpoint;
}



// OBJECT METHODS
// Difference between two points is calculated
// as the distance between the pairs of coordinates.
template <class T>
LAS::Data::Vector3d Point<T>::operator - (Point<T> p) const {
    return LAS::Data::Vector3d(x - p.x, y - p.y, z - p.z);
}

// Sum of point coordinates and a number.
template <class T>
Point<T> Point<T>::operator + (double number) const {
    return Point<T>(x + (T)number, y + (T)number, z + (T)number);
}

// Adding a vector to the point.
template <class T>
Point<T> Point<T>::operator + (LAS::Data::Vector3d v) const {
    return Point<T>(x + (T)v.x, y + (T)v.y, z + (T)v.z);
}


// Difference of point coordinates and a number.
template <class T>
Point<T> Point<T>::operator - (double number) const {
    return Point<T>(x - (T)number, y - (T)number, z - (T)number);
}

// Subtracting a vector to the point.
template <class T>
Point<T> Point<T>::operator - (LAS::Data::Vector3d v) const {
    return Point<T>(x - (T)v.x, y - (T)v.y, z - (T)v.z);
}

// Multiplying of a point and a scalar.
template <class T>
Point<T> Point<T>::operator * (double factor) const {
    return Point<T>((T)factor * x, (T)factor * y, (T)factor * z);
}

// Multiplying of a vector and a point.
template <class T>
Point<T> Point<T>::operator * (LAS::Data::Vector3d v) const {
    return Point<T>((T)v.x * x, (T)v.y * y, (T)v.z * z);
}

// Checking whether the two points are the same.
template <class T>
bool Point<T>::operator == (Point<T> p) const {
    return Tolerance::isInTolerance(x, p.getX(), 0.01) && Tolerance::isInTolerance(y, p.getY(), 0.01) && Tolerance::isInTolerance(z, p.getZ(), 0.01);
}

// Transforming point to vector (Vector3d).
template <class T>
LAS::Data::Vector3d Point<T>::toVec() {
    return LAS::Data::Vector3d(x, y, z);
}


template class Point<int>;
template class Point<float>;
template class Point<double>;
