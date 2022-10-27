#ifndef REFLECTIONSYMMETRY_H
#define REFLECTIONSYMMETRY_H

#include <set>
#include <vector>

#include "localsymmetry.h"
#include "helper_classes/linesegment.h"
#include "helper_classes/plane.h"
#include "helper_classes/positionfromplane.h"


namespace Symmetry {
    // Class for reflection symmetries.
    class ReflectionSymmetry : public LocalSymmetry {
    private:
        // OBJECT VARIABLES
        Plane plane;
        std::vector<LineSegment> lineSegments;
        std::vector<Voxel> voxels;

        // CLASS METHODS
        static double calculateInterestingPercentInLineSegment(const LineSegment& lineSegment);  // Calculating the percent of interesting line segment sections.
        static std::vector<ReflectionSymmetry> findSimpleSymmetries(const std::vector<LineSegment>& lineSegments, const double tolerance);  // Finding simple symmetries.
        static std::vector<std::vector<LineSegment>> splitLineSegmentVectorIntoParts(const std::vector<LineSegment>& lineSegments, const double tolerance);  // Splitting a line segment vector into parts, where the lenghts of line segments is below the tolerance.
        static ReflectionSymmetry * calculateSimpleReflectionSymmetry(LineSegment& l1, LineSegment& l2, const double tolerance);  // Checking whether two line segments represent a reflection symmetry.
        static std::vector<ReflectionSymmetry> mergeSymmetries(std::vector<ReflectionSymmetry>& reflectionSymmetries, const double tolerance);  // Merging reflection symmetries.
        static void getVoxelsInSymmetries(std::vector<ReflectionSymmetry>& reflectionSymmetries);  // Getting voxels for each reflection symmetry.
        static void addInterestingVoxelsToSymmetry(std::vector<ReflectionSymmetry>& reflectionSymmetries);  // Adding interesting voxels back to symmetry (if in symmetry).
        static void removeSmallClusters(std::vector<ReflectionSymmetry>& reflectionSymmetries, const int minimumClusterSize);  // Removing small clusters of symmetries.

        // OBJECT METHODS
        static void removeVoxelsWithoutPairs(std::vector<ReflectionSymmetry>& reflectionSymmetries);  // Removing voxels without their pairs.
        void clusterRecursiveStep(std::set<Voxel>& cluster, std::vector<std::vector<std::vector<Voxel>>>& voxelVector, const int x, const int y, const int z) const;  // Clustering recursive step.
        std::vector<std::set<Voxel>> findClusters() const;  // Finding clusters of symmetry voxels in symmetry.

    public:
        // CONSTRUCTORS
        ReflectionSymmetry(Plane plane, std::vector<LineSegment> lineSegments);    // Default constructor.
        ReflectionSymmetry(const Plane& plane, const std::vector<Voxel>& voxels);  // Constructor with the plane and the voxels.

        // OVERLOADED OPERATORS
        bool operator == (ReflectionSymmetry& s);                // Operator == is used for checking whether the two symmetries are equal.
        ReflectionSymmetry operator &= (ReflectionSymmetry& s);  // Operator &= is used for merging the current symmetry with another.

        // GETTERS AND SETTERS
        const Plane getPlane() const;                                         // Symmetry plane getter.
        const std::vector<LineSegment> getLineSegments() const;               // Line segments getter.
        unsigned int getLineSegmentCount() const;                             // Getting number of line segments.
        void addLineSegment(LineSegment lineSegment);                         // Adding a new line segment.
        const std::vector<Voxel> getVoxels() const;                           // Voxel vector getter.

        // CLASS METHODS
        static std::vector<LineSegment> calculateLineSegmentsBetweenPoints(std::vector<Point<float>> points, const double tolerance);    // Getting line segments between all pairs of points (fully connected graph).
        static std::vector<ReflectionSymmetry> calculateReflectionSymmetries(std::vector<LineSegment> lineSegments, const double tolerance, const int minimumClusterSize);  // Calculating reflection symmetries.
        static std::vector<ReflectionSymmetry> findPartialSymmetries(const std::vector<ReflectionSymmetry>& reflectionSymmetries, const int minSymmetrySize);  // Finding partial symmetries from all reflection symmetries.

        // OBJECT METHODS
        std::vector<PositionFromPlane> calculatePositionsFromPlane(const std::vector<Point<float>>& points, const Plane* plane);  // Calculating the point positions according to a plane.
    };
};

#endif // REFLECTIONSYMMETRY_H
