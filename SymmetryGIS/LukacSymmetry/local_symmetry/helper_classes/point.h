#ifndef POINT_H
#define POINT_H

#include <string>
#include <vector>


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

        // GETTERS AND SETTERS
        const T getX() const;   // X coordinate getter.
        void setX(const T x);   // X coordinate setter.
        const T getY() const;   // Y coordinate getter.
        void setY(const T y);   // Y coordinate setter.
        const T getZ() const;   // Z coordinate getter.
        void setZ(const T z);   // Z coordinate setter.

        // CLASS METHODS
        static double distance(Point<T> v1, Point<T> v2);                                       // Calculation of the Euclidean distance between two points.
        static Point<T> calculateMidpoint(Point<T> p1, Point<T> p2);                            // Calculation of the midpoint of a line segment.
        static Point<double> calculateBarycenterPoint(std::vector<Point<double>>& points);      // Calculation of a barycenter point (centroid).
        static Point<double> crossProduct(const Point<double>& lhs, const Point<double>& rhs);  // Cross product of two vectors.

        // OBJECT METHODS
        std::string toString() const;                          // Converting a point to a string.
        Point<T> operator + (const T& number) const;             // Sum of point coordinates and a number.
        Point<T> operator + (Point<T>& p) const;               // Adding a vector to the point.
        Point<T> operator += (Point<T>& p);                    // Sum of two points.
        Point<T> operator - (const T& number) const;             // Difference of point coordinates and a number.
        Point<T> operator - (const Point<T>& p) const;    // Difference between two points is calculated as the distance between the pairs of coordinates.
        Point<T> operator * (const T& factor) const;             // Multiplying of a point and a scalar.
        Point<T> operator * (Point<T>& p) const;               // Multiplying of a vector and a point.
        Point<T> operator / (const T &factor) const;             // Division of point coordinates and a number.
        Point<T> operator /= (const T &factor);                  // Division of point coordinates and a number.
        bool operator == (Point p) const;                      // Checking whether the two points are the same.
        bool operator != (Point p) const;                      // Checking whether the two points are not the same.
        T dot(const Point<T>& p1) const;                        // Dot product of two vectors.
        double length() const;                                 // Getting length of a vector (point).
        Point<T> normalize();                                  // Vector normalization.
    
        template <typename U>
        Point<U> convert()
        {
            return Point<U>(static_cast<U>(x), static_cast<U>(y), static_cast<U>(z));
        };
    };

    using Vector3d = Point<double>;
    using Vector2d = Point<double>;
    using Vector3f = Point<float>;
    
};

#endif // POINT_H
