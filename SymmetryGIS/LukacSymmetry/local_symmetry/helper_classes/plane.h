#ifndef PLANE_H
#define PLANE_H

#include <string>

#include "point.h"
#include "voxel.h"


namespace Symmetry {
    // Class for plane (ax + by + cz - d == 0).
    class Plane {
    private:
        // OBJECT VARIABLES
        double a;
        double b;
        double c;
        double d;
    public:
        // CONSTRUCTORS
        Plane();                                      // Default constructor with no arguments.
        Plane(Vector3d p, Vector3d v1, Vector3d v2);  // Constructor with two vectors and a point.

        // GETTERS
        double getA() const;         // Coefficient A getter.
        double getB() const;         // Coefficient B getter.
        double getC() const;         // Coefficient C getter.
        double getD() const;         // Coefficient D getter.

        // OVERLOADED OPERATORS
        bool operator == (Plane p) const;  // Checking if the two planes have the same coefficients.

        // CLASS METHODS
        static std::tuple<Vector3d, double, double, double> calculateStartPoint(const Plane& p, const VoxelMesh& vm);  // Calculating the starting point of the plane in a symmetry.

        // OBJECT METHODS
        std::string toString() const;                                                                                              // Converting a plane to string.
        Vector3d calculateNormalVector() const;                                                                                    // Calculating a plane normal vector;
        Vector3d calculateParallelVector() const;                                                                                  // Calculating a vector, parallel to a plane.
        Vector3d calculateZVector() const;                                                                                         // Calculating a vector that goes along the Z axis.
        Point<double>* calculateTopBoundingBoxIntersection(const VoxelMesh& vm) const;                                             // Calculating a point on the plane with Y==max if possible.
        Point<double>* calculateBottomBoundingBoxIntersection(const VoxelMesh& vm) const;                                          // Calculating a point on the plane with Y==min if possible.
        Point<double>* calculateLeftBoundingBoxIntersection(const VoxelMesh& vm) const;                                            // Calculating a point on the plane with X==min if possible.
        Point<double>* calculateRightBoundingBoxIntersection(const VoxelMesh& vm) const;                                           // Calculating a point on the plane with X==max if possible.
        bool isPointOnTheLeftSide(const Point<float>& p, const VoxelMesh& vm) const;                                               // Returning true if a point is on the left side of the vector.
        Point<float> calculateProjectionPoint(const Point<float>& p, const VoxelMesh& vm) const;                                   // Calculating the projection point to the plane.
        double calculateYCoordinateAtX(const double x) const;                                                                      // Calculating the Y coordinate on the plane according to the given X coordinate.
        double calculateXCoordinateAtY(const double y) const;                                                                      // Calculating the X coordinate on the plane according to the given Y coordinate.
        std::vector<size_t> getPointsIndicesOnPlaneAndVoxelEdge(const std::vector<Point<float>>& points, const VoxelMesh& vm) const;  // Getting the indices of the points that lie on the symmetry plane and voxel edge simultaneously.
    };
};

#endif // PLANE_H
