#include <pch.h>
#include "linesegment.h"
#include "tolerance.h"

using namespace Symmetry;



// CONSTRUCTORS
// Constructor with both end points.
LineSegment::LineSegment(const Point<float>& p1, const Point<float>& p2) :
    p1(p1),
    p2(p2)
{}



// GETTERS AND SETTERS
// First point setter.
void LineSegment::setP1(const Point<float>& p1) {
    this->p1.setX(p1.getX());
    this->p1.setY(p1.getY());
    this->p1.setZ(p1.getZ());
}

// First point getter.
Point<float> LineSegment::getP1() const {
    return p1;
}

// Second point setter.
void LineSegment::setP2(const Point<float>& p2) {
    this->p2.setX(p2.getX());
    this->p2.setY(p2.getY());
    this->p2.setZ(p2.getZ());
}

// Second point getter.
Point<float> LineSegment::getP2() const {
    return p2;
}

// Line segment length getter.
float LineSegment::getLength() const {
    return (float)std::sqrt(std::pow(p1.getX() - p2.getX(), 2) + std::pow(p1.getY() - p2.getY(), 2) + std::pow(p1.getZ() - p2.getZ(), 2));
}

// Full quotient getter.
float LineSegment::getFullQuotient() const {
    return fullQuotient;
}

// Full quotient setter.
void LineSegment::setFullQuotient(float fullQuotient) {
    this->fullQuotient = fullQuotient;
}



// CLASS METHODS
// Returning true if the two line segments have an intersection point.
bool LineSegment::doLineSegmentsIntersect(LineSegment& l1, LineSegment& l2, const double tolerance, const bool edgesIncluded) {
    // If the two line segments are parallel and at the same location,
    // they have infinite number of intersections. Yaaaaaayyyy!
    if (l1 == l2) {
        return true;
    }

    // Getting points from line segments.
    Point<float> P1 = l1.getP1();
    Point<float> P2 = l1.getP2();
    Point<float> P3 = l2.getP1();
    Point<float> P4 = l2.getP2();

    // Calculating vectors between points.
    LAS::Data::Vector3d V12 = P2 - P1;
    LAS::Data::Vector3d V34 = P4 - P3;
    LAS::Data::Vector3d V31 = P1 - P3;

    // Coefficient calculation.
    double D = (V12.x * V34.y) - (V12.y * V34.x);
    double A = (V34.x * V31.y) - (V34.y * V31.x);
    double B = (V12.x * V31.y) - (V12.y * V31.x);

    // If D == 0, the line segments are parallel, therefore,
    // they do not have an intersection point.
    if (Tolerance::isInTolerance(D, 0.0, tolerance)) {
        return false;
    }

    // Normalizing coefficients.
    double Ua = A / D;
    double Ub = B / D;

    if (!edgesIncluded &&
        (Tolerance::isInTolerance(Ua, 0, 0.0001) || Tolerance::isInTolerance(Ua, 1, 0.0001) ||
         Tolerance::isInTolerance(Ub, 0, 0.0001) || Tolerance::isInTolerance(Ub, 1, 0.0001)))
    {
        return false;
    }

    // If the two line segment intersect,
    // true is returned as a result.
    if (Ua > -0.0001 && Ua < 1.0001 && Ub > -0.0001 && Ub < 1.0001) {
        return true;
    }

    return false;
}

// Calculation of an intersetion point between the two line segments.
Point<float>* LineSegment::calculateIntersetion(LineSegment& l1, LineSegment& l2, const double tolerance) {
    // Getting points from line segments.
    Point<float> P1 = l1.getP1();
    Point<float> P2 = l1.getP2();
    Point<float> P3 = l2.getP1();
    Point<float> P4 = l2.getP2();

    // Calculating vectors between points.
    LAS::Data::Vector3d V12 = P2 - P1;
    LAS::Data::Vector3d V34 = P4 - P3;
    LAS::Data::Vector3d V31 = P1 - P3;

    // Coefficient calculation.
    double D = (V12.x * V34.y) - (V12.y * V34.x);
    double A = (V34.x * V31.y) - (V34.y * V31.x);
    double B = (V12.x * V31.y) - (V12.y * V31.x);

    // If D == 0, the line segments are parallel, therefore,
    // they do not have an intersection point.
    if (Tolerance::isInTolerance(D, 0.0, tolerance)) {
        return nullptr;
    }

    // Normalizing coefficients.
    double Ua = A / D;
    double Ub = B / D;

    // If the two line segment intersect,
    // true is returned as a result.
    if (Ua >= 0 && Ua <= 1 && Ub >= 0 && Ub <= 1) {
        double x = P1.getX() + Ua * (P2.getX() - P1.getX());
        double y = P1.getY() + Ua * (P2.getY() - P1.getY());
        double z = P1.getZ() + Ua * (P2.getZ() - P1.getZ());

        return new Point<float>((float)x, (float)y, (float)z);
    }

    return nullptr;
}



// OBJECT METHODS
// Calculating the center point of the line segment.
Point<float> LineSegment::getCenterPoint() const {
    return (
        Point<float>(
            (p1.getX() + p2.getX()) / 2,
            (p1.getY() + p2.getY()) / 2,
            (p1.getZ() + p2.getZ()) / 2
        )
    );
}

// Checking whether the two line segments are equal.
bool LineSegment::operator == (const LineSegment& ls) const {
    return (
        (p1 == ls.getP1() && p2 == ls.getP2()) ||
        (p1 == ls.getP2() && p2 == ls.getP1())
    );
}
