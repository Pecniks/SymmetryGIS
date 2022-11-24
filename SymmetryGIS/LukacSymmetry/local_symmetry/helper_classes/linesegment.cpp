#include <pch.h>
#include <cmath>
#include <numbers>
#include <sstream>

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
    Vector3f V12 = P2 - P1;
    Vector3f V34 = P4 - P3;
    Vector3f V31 = P1 - P3;

    // Coefficient calculation.
    auto D = (V12.getX() * V34.getY()) - (V12.getY() * V34.getX());
    auto A = (V34.getX() * V31.getY()) - (V34.getY() * V31.getX());
    auto B = (V12.getX() * V31.getY()) - (V12.getY() * V31.getX());

    // If D == 0, the line segments are parallel, therefore,
    // they do not have an intersection point.
    if (Tolerance::isInTolerance(D, 0.0, tolerance)) {
        return false;
    }

    // Normalizing coefficients.
    auto Ua = A / D;
    auto Ub = B / D;

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
    Vector3f V12 = P2 - P1;
    Vector3f V34 = P4 - P3;
    Vector3f V31 = P1 - P3;

    // Coefficient calculation.
    auto D = (V12.getX() * V34.getY()) - (V12.getY() * V34.getX());
    auto A = (V34.getX() * V31.getY()) - (V34.getY() * V31.getX());
    auto B = (V12.getX() * V31.getY()) - (V12.getY() * V31.getX());

    // If D == 0, the line segments are parallel, therefore,
    // they do not have an intersection point.
    if (Tolerance::isInTolerance(D, 0.0, tolerance)) {
        return nullptr;
    }

    // Normalizing coefficients.
    auto Ua = A / D;
    auto Ub = B / D;

    // If the two line segment intersect,
    // true is returned as a result.
    if (Ua >= 0 && Ua <= 1 && Ub >= 0 && Ub <= 1) {
        auto x = P1.getX() + Ua * (P2.getX() - P1.getX());
        auto y = P1.getY() + Ua * (P2.getY() - P1.getY());
        auto z = P1.getZ() + Ua * (P2.getZ() - P1.getZ());

        return new Point<float>(x, y, z);
    }

    return nullptr;
}



// OBJECT METHODS
// Converting line segment to string.
std::string LineSegment::toString() const {
    std::stringstream ss;
    ss << "LineSegment(" << p1.toString() << ", " << p2.toString() << ")";
    return ss.str();
}

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

// Checking if line segment is smaller than the other.
bool LineSegment::operator < (const LineSegment& ls) const {
    return p1.getX() < ls.p1.getX() || p1.getY() < ls.p1.getY() || p1.getZ() < ls.p1.getZ() ||
           p2.getX() < ls.p2.getX() || p2.getY() < ls.p2.getY() || p2.getZ() < ls.p2.getZ();
}

// Calculating the angle between the two line segments.
double LineSegment::calculateAngle(const LineSegment& ls) const {
    auto v1 = p2 - p1;
    auto v2 = ls.p2 - ls.p1;

    auto dot = (float)(v1.getX() * v2.getX()) + (float)(v1.getY() * v2.getY());
    auto det = (float)(v1.getX() * v2.getY()) - (float)(v1.getY() * v2.getX());
    auto angle = std::atan2(det, dot);

    // Angle lies inside the interval [0, 2 * PI].
    //return static_cast<double>(angle >= 0 ? angle : angle + M_PI * 2);
    return static_cast<double>(angle >= 0 ? angle : angle + std::numbers::pi * 2);
}
