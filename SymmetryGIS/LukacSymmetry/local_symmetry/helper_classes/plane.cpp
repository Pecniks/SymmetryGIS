#include <pch.h>
#include <cmath>
#include <iomanip>

#include "plane.h"
#include "tolerance.h"

using namespace Symmetry;



// CONSTRUCTORS
// Default constructor with no arguments.
Plane::Plane() {
    a = b = c = d = INT_MAX;
}

// Default constructor with two vectors and a point.
Plane::Plane(Vector3d p, Vector3d v1, Vector3d v2) {
    // Calculating a, b, c and d coefficients.
    // According to Linear Algebra (TM), (a, b, c) represents the plane normal vector.
    Vector3d normal = Vector3d::crossProduct(v1, v2);
    a = normal.getX();
    b = normal.getY();
    c = normal.getZ();
    d = p.dot(normal);

    // To make our lives easier, A coefficient should always be >=0.
    if (a < 0) {
        a = -a;
        b = -b;
        c = -c;
        d = -d;
    }
}



// GETTERS
// Coefficient A getter.
double Plane::getA() const {
    return a;
}

// Coefficient B getter.
double Plane::getB() const {
    return b;
}

// Coefficient C getter.
double Plane::getC() const {
    return c;
}

// Coefficient D getter.
double Plane::getD() const {
    return d;
}



// OVERLOADED OPERATORS
// Checking if the two planes have the same coefficients.
bool Plane::operator == (Plane p) const {
    return (
        (a == p.a || a == -p.a) &&
        (b == p.b || b == -p.b) &&

        (c == p.c || c  == -p.c) &&
        (d == p.d || d  == -p.d)
    );
}



// CLASS METHODS
// Calculating the starting point of the plane in a symmetry.
std::tuple<Vector3d, double, double, double> Plane::calculateStartPoint(const Plane& p, const VoxelMesh& vm) {
    // Calculating the intersections betweeen the symmetry plane and the bounding box.
    Point<double>* pTopBbox = p.calculateTopBoundingBoxIntersection(vm);
    Point<double>* pBottomBbox = p.calculateBottomBoundingBoxIntersection(vm);
    Point<double>* pLeftBbox = p.calculateLeftBoundingBoxIntersection(vm);
    Point<double>* pRightBbox = p.calculateRightBoundingBoxIntersection(vm);

    // Setting the position of a plane.
    Vector3d position;
    const double RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE = 0.5;
    double distance = 0.0;
    double distanceX = 0.0;
    double distanceY = 0.0;
    auto planeY = p.calculateParallelVector().getY();

    // Positioning the plane according to all the posibilities (8).
    if (planeY < 0) {
        // Option 1: left and bottom bounding box intersections.
        if (pLeftBbox != nullptr && pBottomBbox != nullptr) {
            // Calculating the distances (by X, by Y, and total).
            distanceX = pBottomBbox->getX() - pLeftBbox->getX();
            distanceY = pLeftBbox->getY() - pBottomBbox->getY();
            distance = (2 * RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE + 1) * sqrt(pow(distanceX, 2) + pow(distanceY, 2));

            // Positioning the plane.
            position.setX(pLeftBbox->getX() - RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceX);
            position.setY(pLeftBbox->getY() + RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceY);
            position.setZ(vm.minZ - (vm.deltaZ / 2));
        }
        // Option 2: top and bottom bounding box intersections.
        else if (pTopBbox != nullptr && pBottomBbox != nullptr) {
            // Calculating the distances (by X, by Y, and total).
            distanceX = pBottomBbox->getX() - pTopBbox->getX();
            distanceY = pTopBbox->getY() - pBottomBbox->getY();
            distance = (2 * RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE + 1) * sqrt(pow(distanceX, 2) + pow(distanceY, 2));

            // Positioning the plane.
            position.setX(pTopBbox->getX() - RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceX);
            position.setY(pTopBbox->getY() + RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceY);
            position.setZ(vm.minZ - (vm.deltaZ / 2));
        }
        // Option 3: top and right bounding box intersections.
        else if (pTopBbox != nullptr && pRightBbox != nullptr) {
            // Calculating the distances (by X, by Y, and total).
            distanceX = pRightBbox->getX() - pTopBbox->getX();
            distanceY = pTopBbox->getY() - pRightBbox->getY();
            distance = (2 * RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE + 1) * sqrt(pow(distanceX, 2) + pow(distanceY, 2));

            // Positioning the plane.
            position.setX(pTopBbox->getX() - RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceX);
            position.setY(pTopBbox->getY() + RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceY);
            position.setZ(vm.minZ - (vm.deltaZ / 2));
        }
        // Option 4: top and right bounding box intersections.
        else if (pLeftBbox != nullptr && pRightBbox != nullptr) {
            // Calculating the distances (by X, by Y, and total).
            distanceX = pRightBbox->getX() - pLeftBbox->getX();
            distanceY = pLeftBbox->getY() - pRightBbox->getY();
            distance = (2 * RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE + 1) * sqrt(pow(distanceX, 2) + pow(distanceY, 2));

            // Positioning the plane.
            position.setX(pLeftBbox->getX() - RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceX);
            position.setY(pLeftBbox->getY() + RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceY);
            position.setZ(vm.minZ - (vm.deltaZ / 2));
        }
    }
    else {
        // Option 5: bottom and right bounding box intersections.
        if (pBottomBbox != nullptr && pRightBbox != nullptr) {
            // Calculating the distances (by X, by Y, and total).
            distanceX = pRightBbox->getX() - pBottomBbox->getX();
            distanceY = pRightBbox->getY() - pBottomBbox->getY();
            distance = (2 * RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE + 1) * sqrt(pow(distanceX, 2) + pow(distanceY, 2));

            // Positioning the plane.
            position.setX(pRightBbox->getX() + RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceX - 2 * distanceX);
            position.setY(pRightBbox->getY() - RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceY - distanceY);
            position.setZ(vm.minZ - (vm.deltaZ / 2));
        }
        // Option 6: bottom and top bounding box intersections.
        else if (pBottomBbox != nullptr && pTopBbox != nullptr) {
            // Calculating the distances (by X, by Y, and total).
            distanceX = pTopBbox->getX() - pBottomBbox->getX();
            distanceY = pTopBbox->getY() - pBottomBbox->getY();
            distance = (2 * RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE + 1) * sqrt(pow(distanceX, 2) + pow(distanceY, 2));

            // Positioning the plane.
            position.setX(pBottomBbox->getX() + RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceX - distanceX);
            position.setY(pBottomBbox->getY() - RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceY);
            position.setZ(vm.minZ - (vm.deltaZ / 2));
        }
        // Option 7: left and top bounding box intersections.
        else if (pLeftBbox != nullptr && pTopBbox != nullptr) {
            // Calculating the distances (by X, by Y, and total).
            distanceX = pTopBbox->getX() - pLeftBbox->getX();
            distanceY = pTopBbox->getY() - pLeftBbox->getY();
            distance = (2 * RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE + 1) * sqrt(pow(distanceX, 2) + pow(distanceY, 2));

            // Positioning the plane.
            position.setX(pLeftBbox->getX() + RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceX - distanceX);
            position.setY(pLeftBbox->getY() - RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceY);
            position.setZ(vm.minZ - (vm.deltaZ / 2));
        }
        // Option 8: left and right bounding box intersections.
        else if (pLeftBbox != nullptr && pRightBbox != nullptr) {
            // Calculating the distances (by X, by Y, and total).
            distanceX = pRightBbox->getX() - pLeftBbox->getX();
            distanceY = pRightBbox->getY() - pLeftBbox->getY();
            distance = (2 * RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE + 1) * sqrt(pow(distanceX, 2) + pow(distanceY, 2));

            // Positioning the plane.
            position.setX(pLeftBbox->getX() + RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceX - distanceX);
            position.setY(pLeftBbox->getY() - RELATIVE_LENGTH_OUTSIDE_BBOX_EACH_SIDE * distanceY);
            position.setZ(vm.minZ - (vm.deltaZ / 2));
        }
    }

    // Deleting all the dynamically allocated variables in
    // order to prevent nasty nasty nasty memory leaks.
    delete pTopBbox;
    delete pBottomBbox;
    delete pLeftBbox;
    delete pRightBbox;

    return std::make_tuple(position, distance, distanceX, distanceY);
}



// OBJECT METHODS
// Converting a plane to string.
std::string Plane::toString() const {
    std::stringstream ss;
    ss << std::setprecision(2);  // Setting the maximum of 2 decimal places.

    // A coefficient output.
    ss << a << "x";

    // B coefficient output.
    if (b >= 0) {
        ss << "+" << std::abs(b) << "y";
    }
    else {
        ss << b << "y";
    }

    // C coefficient output.
    if (c >= 0) {
        ss << "+" << std::abs(c) << "z=";
    }
    else {
        ss << c << "z=";
    }

    // D coefficient output.
    if (d >= 0) {
        ss << std::abs(d);
    }
    else {
        ss << d;
    }

    return ss.str();
}

// Calculating a plane normal vector;
Vector3d Plane::calculateNormalVector() const {
    return Vector3d(a, b, c);
}

// Calculating a vector, parallel to a plane.
Vector3d Plane::calculateParallelVector() const {
    Vector3d v = Vector3d(-b, a, 0);  // Calculating a parallel vector.

    // To make our lives easier again, the X component
    // of the vector should be >=0. Always.
    if (v.getX() < 0) {
        v.setX(-v.getX());
        v.setY(-v.getY());
    }

    return v;
}

// Calculating a vector that goes along the Z axis.
Vector3d Plane::calculateZVector() const {
    return Vector3d(0, 0, 1);
}

// Calculating a point on the plane with Y==max if possible.
Point<double>* Plane::calculateTopBoundingBoxIntersection(const VoxelMesh& vm) const {
    Vector3d v = calculateParallelVector();  // Calculating a parallel vector.

    // If the plane is constant along the Y axis, it
    // will never reach the top side of the bounding box.
    if (v.getY() == 0 && d != 0) {
        return nullptr;
    }

    // Calculating the coordinates of the intersection point.
    double x = (d - b * vm.maxY) / a;
    double y = vm.maxY;
    double z = 0;

    // If X lies outside the bounding box, there is no intersection.
    if (x < vm.minX || x > vm.maxX) {
        return nullptr;
    }

    return new Point<double>(x, y, z);
}

// Calculating a point on the plane with Y==min if possible.
Point<double>* Plane::calculateBottomBoundingBoxIntersection(const VoxelMesh& vm) const {
    Vector3d v = calculateParallelVector();  // Calculating a parallel vector.

    // If the plane is constant along the Y axis, it
    // will never reach the bottom side of the bounding box.
    if (v.getY() == 0 && d != 0) {
        return nullptr;
    }

    // Calculating the coordinates of the intersection point.
    double x = (d - b * vm.minY) / a;
    double y = vm.minY;
    double z = 0;

    // If X lies outside the bounding box, there is no intersection.
    if (x < vm.minX || x > vm.maxX) {
        return nullptr;
    }

    return new Point<double>(x, y, z);
}

// Calculating a point on the plane with X==min if possible.
Point<double>* Plane::calculateLeftBoundingBoxIntersection(const VoxelMesh& vm) const {
    Vector3d v = calculateParallelVector();  // Calculating a parallel vector.

    // If the plane is constant along the X axis, it
    // will never reach the left side of the bounding box.
    if (v.getX() == 0 && d != 0) {
        return nullptr;
    }

    // Calculating the coordinates of the intersection point.
    double x = vm.minX;
    double y = (d - a * vm.minX) / b;
    double z = 0;

    // If Y lies outside the bounding box, there is no intersection.
    if (y < vm.minY || y > vm.maxY) {
        return nullptr;
    }

    return new Point<double>(x, y, z);
}

// Calculating a point on the plane with X==max if possible.
Point<double>* Plane::calculateRightBoundingBoxIntersection(const VoxelMesh& vm) const {
    // The Y component of the point is calculated as d/b.
    Vector3d v = calculateParallelVector();  // Calculating a parallel vector.

    // If the plane is constant along the X axis, it
    // will never reach the left side of the bounding box.
    if (v.getX() == 0 && d != 0) {
        return nullptr;
    }

    // Calculating the coordinates of the intersection point.
    double x = vm.maxX;
    double y = (d - a * vm.maxX) / b;
    double z = 0;

    // If Y lies outside the bounding box, there is no intersection.
    if (y < vm.minY || y > vm.maxY) {
        return nullptr;
    }

    return new Point<double>(x, y, z);
}

// Returning true if a point is on the left side of the plane.
bool Plane::isPointOnTheLeftSide(const Point<float>& p, const VoxelMesh& vm) const {
    // Calculating top, bottom and leftbounding box intersections.
    // Since all planes intersect with the bounding box at least
    // twice, 3 intersection points are enough for sure.
    Point<double>* pTop = calculateTopBoundingBoxIntersection(vm);        // Calculating top intersection.
    Point<double>* pBottom = calculateBottomBoundingBoxIntersection(vm);  // Calculating bottom intersection.
    Point<double>* pLeft = calculateLeftBoundingBoxIntersection(vm);      // Calculating left intersection.
    double cross = 0.0;  // Cross product between two vectors.

    // Calculation of the side with a help of the top intersection point.
    Vector3d parallel = calculateParallelVector();
    if (pTop != nullptr) {
        Point<double> p1 = *pTop - parallel;
        Vector2d v1 = (Vector2d(calculateParallelVector().getX(), calculateParallelVector().getY(), 0)).normalize();
        Vector2d v2 = (Vector2d(p.getX() - p1.getX(), p.getY() - p1.getY(), 0)).normalize();
        cross = v1.getX() * v2.getY() - v2.getX() * v1.getY();
    }
    // Calculation of the side with a help of the bottom intersection point.
    else if (pBottom != nullptr) {
        Point<double> p1 = *pBottom - parallel;
        Vector2d v1 = (Vector2d(calculateParallelVector().getX(), calculateParallelVector().getY(), 0)).normalize();
        Vector2d v2 = (Vector2d(p.getX() - p1.getX(), p.getY() - p1.getY(), 0)).normalize();
        cross = v1.getX() * v2.getY() - v2.getX() * v1.getY();
    }
    // Calculation of the side with a help of the left intersection point.
    else {
        Point<double> p1 = *pLeft - parallel;
        Vector2d v1 = (Vector2d(calculateParallelVector().getX(), calculateParallelVector().getY(), 0)).normalize();
        Vector2d v2 = (Vector2d(p.getX() - p1.getX(), p.getY() - p1.getY(), 0)).normalize();
        cross = v1.getX() * v2.getY() - v2.getX() * v1.getY();
    }

    // Deleting points.
    delete pTop;
    delete pBottom;
    delete pLeft;

    // If cross product is <0, the point lies on the left side of the plane.
    return cross < 0;
}

// Calculating the projection point to the plane.
Point<float> Plane::calculateProjectionPoint(const Point<float>& p, const VoxelMesh& vm) const {
    // Calculating top, bottom and leftbounding box intersections.
    Point<double>* p1 = calculateTopBoundingBoxIntersection(vm);     // Calculating the top intersection.
    Point<double>* p2 = calculateBottomBoundingBoxIntersection(vm);  // Calculating the bottom intersection.
    Point<double>* p3 = calculateLeftBoundingBoxIntersection(vm);    // Calculating the left intersection.
    Point<double>* p4 = calculateRightBoundingBoxIntersection(vm);   // Calculating the right intersection.
    Point<float> projection;

    // Calculation of the projection point with a help of the top intersection point.
    if (p1 != nullptr) {
        // Base vector calculation.
        Vector3d v1(calculateParallelVector());
        Vector3d v2(p1->getX() - p.getX() + vm.minX, p1->getY() - p.getY() + vm.minY, 0);
        Vector3d vn = Vector3d(p1->getX() - p.getX() + vm.minX, p1->getY() - p.getY() + vm.minY, 0).normalize();
        double length = v2.length();

        // Projection calculation.
        double dot = -v1.dot(vn);
        projection.setX((float)(p1->getX() + dot * v1.getX() * length + vm.minX));
        projection.setY((float)(p1->getY() + dot * v1.getY() * length + vm.minX));
        projection.setZ(p.getZ());
    }
    // Calculation of the projection point with a help of the bottom intersection point.
    else if (p2 != nullptr) {
        // Base vector calculation.
        Vector3d v1(calculateParallelVector());
        Vector3d v2(p2->getX() - p.getX() + vm.minX, p2->getY() - p.getY() + vm.minY, 0);
        Vector3d vn = Vector3d(p2->getX() - p.getX() + vm.minX, p2->getY() - p.getY() + vm.minY, 0).normalize();
        double length = v2.length();

        // Projection calculation.
        double dot = -v1.dot(vn);
        projection.setX((float)(p2->getX() + dot * v1.getX() * length + vm.minX));
        projection.setY((float)(p2->getY() + dot * v1.getY() * length + vm.minY));
        projection.setZ(p.getZ());
    }
    // Calculation of the projection point with a help of the left intersection point.
    else if (p3 != nullptr) {
        // Base vector calculation.
        Vector3d v1(calculateParallelVector());
        Vector3d v2(p3->getX() - p.getX() + vm.minX, p3->getY() - p.getY() + vm.minY, 0);
        Vector3d vn = Vector3d(p3->getX() - p.getX() + vm.minX, p3->getY() - p.getY() + vm.minY, 0).normalize();
        double length = v2.length();

        // Projection calculation.
        double dot = std::abs(v1.dot(vn));
        projection.setX((float)(p3->getX() + dot * v1.getX() * length + vm.minX));
        projection.setY((float)(p3->getY() + dot * v1.getY() * length + vm.minY));
        projection.setZ(p.getZ());
    }
    // Calculation of the projection point with a help of the right intersection point.
    else if (p4 != nullptr) {
        // Base vector calculation.
        Vector3d v1(calculateParallelVector());
        Vector3d v2(p4->getX() - p.getX() + vm.minX, p4->getY() - p.getY() + vm.minY, 0);
        Vector3d vn = Vector3d(p4->getX() - p.getX() + vm.minX, p4->getY() - p.getY() + vm.minY, 0).normalize();
        double length = v2.length();

        // Projection calculation.
        double dot = std::abs(v1.dot(vn));
        projection.setX((float)(p4->getX() - dot * v1.getX() * length + vm.minX));
        projection.setY((float)(p4->getY() - dot * v1.getY() * length + vm.minY));
        projection.setZ(p.getZ());
    }

    // Deleting points.
    delete p1;
    delete p2;
    delete p3;
    delete p4;

    return projection;
}

// Calculating the Y coordinate on the plane according to the given X coordinate.
double Plane::calculateYCoordinateAtX(const double x) const {
    // Y can be calculated pretty easily from the plane equation (ax+by+cz=d).
    double y = (d - a * x) / b;
    return y;
}

// Calculating the X coordinate on the plane according to the given Y coordinate.
double Plane::calculateXCoordinateAtY(const double y) const {
    // X can be calculated pretty easily from the plane equation (ax+by+cz=d).
    double x = (d - b * y) / a;
    return x;
}

// Getting the indices of the points that lie on the symmetry plane and voxel edge simultaneously.
std::vector<size_t> Plane::getPointsIndicesOnPlaneAndVoxelEdge(const std::vector<Point<float>>& points, const VoxelMesh& vm) const {
    std::vector<size_t> indices;

    for (size_t i = 0; i < points.size(); i++) {
        // If the projection point and the point do not have
        // the same coordinates, the point does not lie on the plane.
        const Point<float> projection = calculateProjectionPoint(points[i], vm);
        if (points[i] != projection) {
            continue;
        }

        // If the quotient between the point X coordinate
        if (!Tolerance::isInTolerance(fmod(points[i].getX() / vm.voxelSideSize, 1), 0.0, 0.0001) &&
            !Tolerance::isInTolerance(fmod(points[i].getY() / vm.voxelSideSize, 1), 0.0, 0.0001)
        )
        {
            continue;
        }

        indices.push_back(i);
    }

    return indices;
}
