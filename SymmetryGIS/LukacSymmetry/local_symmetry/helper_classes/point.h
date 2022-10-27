#ifndef POINT_H
#define POINT_H

#include "LAS/Data/nVector.h"


namespace Symmetry {
    // Point with X, Y and Z coordinate.
    template <class T>
    class Point {
    private:
        // CLASS VARIABLES
        T x;  // X coordinate.
        T y;  // Y coordinate.
        T z;  // Z coordinate.
    public:
        // CONSTRUCTORS
        Point();                              // Default constructor without parameters.
        Point(T x, T y, T z);                 // Constructor with X, Y and Z coordinates.
        Point(const Point& p);                // Copy constructor.
        Point(const LAS::Data::Vector3d& v);  // Copy constructor from vec3.

        // GETTERS AND SETTERS
        const T getX() const;   // X coordinate getter.
        void setX(const T x);   // X coordinate setter.
        const T getY() const;   // Y coordinate getter.
        void setY(const T y);   // Y coordinate setter.
        const T getZ() const;   // Z coordinate getter.
        void setZ(const T z);   // Z coordinate setter.

        // CLASS METHODS
        static double distance(Point<T> v1, Point<T> v2);             // Calculation of the Euclidean distance between two points.
        static Point<T> calculateMidpoint(Point<T> p1, Point<T> p2);  // Calculation of the midpoint of a line segment.

        // OBJECT METHODS
        LAS::Data::Vector3d operator - (Point p) const;        // Difference between two points is calculated as the distance between the pairs of coordinates.
        Point<T> operator + (double number) const;             // Sum of point coordinates and a number.
        Point<T> operator + (LAS::Data::Vector3d v) const;     // Adding a vector to the point.
        Point<T> operator - (double number) const;             // Difference of point coordinates and a number.
        Point<T> operator - (LAS::Data::Vector3d v) const;     // Subtracting a vector to the point.
        Point<T> operator * (double factor) const;             // Multiplying of a point and a scalar.
        Point<T> operator * (LAS::Data::Vector3d v) const;     // Multiplying of a vector and a point.
        bool operator == (Point p) const;                      // Checking whether the two points are the same.
        LAS::Data::Vector3d toVec();                           // Transforming point to vector (Vector3d).
    };
};

#endif // POINT_H
