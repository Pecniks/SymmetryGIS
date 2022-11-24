#ifndef LINESEGMENT_H
#define LINESEGMENT_H

#include <string>
#include <vector>

#include "point.h"


namespace Symmetry {
    // Line segment with two end points.
    class LineSegment {
    private:
        // CLASS VARIABLES
        Point<float> p1;      // First end point that limits the line segment.
        Point<float> p2;      // Second end point that limits the line segment.
        float fullQuotient;   // Quotient of full voxels against all voxels.
    public:
        // CONSTRUCTORS
        LineSegment(const Point<float>& p1, const Point<float>& p2);  // Constructor with both end points.

        // GETTERS AND SETTERS
        void setP1(const Point<float>& p1);        // First point setter.
        Point<float> getP1() const;                // First point getter.
        void setP2(const Point<float>& p2);        // Second point setter.
        Point<float> getP2() const;                // Second point getter.
        float getLength() const;                   // Line segment length getter.
        float getFullQuotient() const;             // Full quotient getter.
        void setFullQuotient(float fullQuotient);  // Full quotient setter.

        // CLASS METHODS
        static bool doLineSegmentsIntersect(LineSegment& l1, LineSegment& l2, const double tolerance, const bool edgesIncluded);  // Returning true if the two line segments have an intersection point.
        static Point<float>* calculateIntersetion(LineSegment& l1, LineSegment& l2, const double tolerance);                      // Calculation of an intersetion point between the two line segments.

        // OBJECT METHODS
        std::string toString() const;                        // Converting line segment to string.
        Point<float> getCenterPoint() const;                 // Calculating the center point of the line segment.
        bool operator == (const LineSegment& ls) const;      // Checking whether the two line segments are equal.
        bool operator < (const LineSegment& ls) const;       // Checking if line segment is smaller than the other.
        double calculateAngle(const LineSegment& ls) const;  // Calculating the angle between the two line segments.
    };
};

#endif // LINESEGMENT_H
